#include "constants.h"
#include "api.h"
#include "endeffector.h"
#include "elevator.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace endeffector{
    pros::Motor pivotA(constants::endEffectorPivotAPort, constants::endEffectorGears);
    pros::Motor pivotB(constants::endEffectorPivotBPort, constants::endEffectorGears);
    pros::Motor claw(constants::endEffectorClawPort, constants::endEffectorGears);

    //a preload home zeroes the claw where it stalls on the Pin, not at its stop. this shifts every claw target so the rest
    //of the code keeps using the normal degrees. 0 after an empty home
    static double clawOffsetDeg {0};

    void init(){
        pivotA.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        pivotB.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }

    //closes the claw at a low voltage and zeroes it as soon as it stalls (after the grace time, so it isn't mistaken for a
    //stall while it spins up), or at the timeout if it never does. with a preload Pin the stall is on the Pin, so the zero
    //is shifted by the Pin grip
    //minTravelDeg: a stall only counts once the claw has closed this far. a claw starting open can sit still for a moment
    //before it breaks free, which would otherwise read as a stall and zero it while open
    static void homeClaw(bool preload, double minTravelDeg = 0){
        uint32_t start {pros::millis()};
        double startPosition {claw.get_position()};
        bool stalled {false};
        uint32_t slowSince {0};
        //on a Pin it closes harder, so the stall point is a firm grip that the brake then holds
        claw.move_voltage(-(preload ? constants::endEffectorClawClampMv : constants::endEffectorClawHomeMv));
        while(true){
            uint32_t elapsed {pros::millis() - start};
            bool slow {elapsed > constants::endEffectorClawHomeGraceMs
                       && startPosition - claw.get_position() >= minTravelDeg
                       && std::abs(claw.get_actual_velocity()) < constants::endEffectorClawHomeStallRpm};
            //one slow reading mid-close zeroed the claw while it was still closing, so it has to stay slow for 100 ms
            if(!slow){
                slowSince = 0;
            }
            else if(slowSince == 0){
                slowSince = pros::millis();
            }
            stalled = slowSince != 0 && pros::millis() - slowSince >= 100;
            if(stalled || elapsed > constants::endEffectorClawHomeTimeoutMs){
                break;
            }
            pros::delay(20);
        }
        std::printf("claw home (%s): %s after %u ms, closed %.1f deg to the zero\n", preload ? "on the Pin" : "empty",
                    stalled ? "stalled" : "timed out", static_cast<unsigned>(pros::millis() - start),
                    startPosition - claw.get_position());
        claw.brake();
        claw.tare_position();
        clawOffsetDeg = preload ? -constants::endEffectorClawPinGripDeg : 0;
    }

    static bool clawRehoming {false};

    void rehomeClaw(std::function<bool()> startWhen){
        clawRehoming = true;
        pros::Task([startWhen]{
            std::uint32_t start {pros::millis()};
            while(!startWhen() && pros::millis() - start < 3000){
                pros::delay(10);
            }
            std::printf("claw rehome: starting after %u ms of waiting, claw at %.1f deg\n",
                        static_cast<unsigned>(pros::millis() - start), claw.get_position() - clawOffsetDeg);
            //starts open, so it has to close most of the way before a stall counts
            homeClaw(false, 0.75 * (claw.get_position() - clawOffsetDeg));
            clawRehoming = false;
        });
    }

    void waitForClawHome(){
        while(clawRehoming){
            pros::delay(10);
        }
    }

    //motors only run once the robot is enabled, so call this at the start of autonomous/opcontrol (alongside the elevator home).
    //homes the claw first, then the pivot. the claw closes onto its stop, then the pivot (which starts at its highest
    //point) drives up into its top stop, each until it stalls (or times out), then zeroes there. a mechanism that is already
    //homed is skipped. with a preload Pin the claw is left alone (clampPreload() homes it on the move)
    static bool clawHomed {false};
    static bool pivotHomed {false};

    void forgetHome(){
        clawHomed = false;
        pivotHomed = false;
    }

    void homeBlocking(bool preload){

        //with a preload the claw home waits until the end (below), so it happens while the robot is already driving
        if(!clawHomed && !preload){
            homeClaw(false);
            clawHomed = true;
        }

        if(preload){
            //push the claw shut on the Pin before the pivot moves, and keep pushing through the pivot home and stow: the
            //brake alone let the Pin fling out. clampPreload() takes over from here, so it stays powered until then
            claw.move_voltage(-constants::endEffectorClawClampMv);
        }

        if(!pivotHomed){
            uint32_t start {pros::millis()};
            pivotA.move_voltage(constants::endEffectorPivotHomeMv);
            pivotB.move_voltage(constants::endEffectorPivotHomeMv);
            while(true){
                uint32_t elapsed {pros::millis() - start};
                //both pivot motors must be slow, so one slipping motor can't end the home early
                double speed {std::max(std::abs(pivotA.get_actual_velocity()), std::abs(pivotB.get_actual_velocity()))};
                bool stalled {elapsed > constants::endEffectorPivotHomeGraceMs && speed < constants::endEffectorPivotHomeStallRpm};
                if(stalled || elapsed > constants::endEffectorPivotHomeTimeoutMs){
                    break;
                }
                pros::delay(20);
            }
            pivotA.brake();
            pivotB.brake();
            pivotA.tare_position();
            pivotB.tare_position();
            pivotHomed = true;
        }

        //go to the stow position now that the zeroes are known
        if(clawHomed && !preload){
            closeClaw();
        }
        if(pivotHomed){
            pivotTo(constants::endEffectorPivotStowDeg);

            //wait for the pivot to reach stow so the elevator (which starts moving once homing returns) doesn't hit it on the way
            uint32_t start {pros::millis()};
            while(std::abs((pivotA.get_position() + pivotB.get_position()) / 2 - constants::endEffectorPivotStowDeg) > constants::endEffectorPivotStowToleranceDeg
                  && pros::millis() - start < constants::endEffectorPivotStowTimeoutMs){
                pros::delay(20);
            }
        }
    }

    void clampPreload(){
        clawRehoming = true;
        clawHomed = true;
        pros::Task([]{
            homeClaw(true);
            //the brake alone didn't keep the Pin from flinging out when the pivot swings down, so keep pushing the claw shut
            //until the swing is done
            claw.move_voltage(-constants::endEffectorClawClampMv);
            pros::delay(constants::autonClawMs);
            pivotTo(constants::endEffectorPivotHorizontalDeg);
            pros::delay(constants::autonPivotMs);
            //wiggle the claw open a couple degrees and shut again, so a Pin stuck at an awkward angle settles square. the
            //zero is the Pin grip, so + is open; shutting is the clamp voltage, which the Pin stops
            for(int i {0}; i < constants::endEffectorClawWiggles; ++i){
                claw.move_absolute(constants::endEffectorClawWiggleDeg, constants::endEffectorClawRpm);
                pros::delay(constants::endEffectorClawWiggleMs);
                claw.move_voltage(-constants::endEffectorClawClampMv);
                pros::delay(constants::endEffectorClawWiggleMs);
            }
            claw.brake();
            clawRehoming = false;
        });
    }

    void openClaw(){
        claw.move_absolute(constants::endEffectorClawOpenDeg + clawOffsetDeg, constants::endEffectorClawRpm);
    }

    void closeClaw(){
        claw.move_absolute(clawOffsetDeg, constants::endEffectorClawRpm);
    }

    void pivotTo(double deg){
        pivotA.move_absolute(deg, constants::endEffectorPivotRpm);
        pivotB.move_absolute(deg, constants::endEffectorPivotRpm);
    }

    //bench test, call every loop. right stick Y drives both pivots. holding R1 opens the claw, moves the pivot to horizontal and
    //the elevator to its intake height; releasing R1 closes the claw.    //if the pivots fight each other, negate endEffectorPivotBPort in constants.h
    void test(){
        //only take over the pivot from a move_absolute while the stick is used, or when it was just released
        static bool stickActive {false};
        int pivot {constants::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)};
        if(std::abs(pivot) >= 10){
            stickActive = true;
            pivotA.move_voltage(pivot * constants::endEffectorTestPivotMvPerStick);
            pivotB.move_voltage(pivot * constants::endEffectorTestPivotMvPerStick);
        }
        else if(stickActive){
            stickActive = false;
            pivotA.brake();
            pivotB.brake();
        }

        if(constants::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) != 0){
            openClaw();
            pivotTo(constants::endEffectorPivotHorizontalDeg);
            elevator::requestIntake();
        }
        else if(constants::master.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_R1) != 0){
            closeClaw();
        }

        //R2 stows: claw closed, pivot up to the stow position
        if(constants::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2) != 0){
            closeClaw();
            pivotTo(constants::endEffectorPivotStowDeg);
        }
    }

}
