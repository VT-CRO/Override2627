#include "subsystems/endeffector.h"
#include "config/auton.h"
#include "config/endeffector.h"
#include "config/ports.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace cfg = config::endeffector;

EndEffector::EndEffector(pros::v5::Controller& master, Elevator& elevator)
    : master(master),
      elevator(elevator),
      pivotA(config::ports::pivotAPort, config::ports::endEffectorGearRatio),
      pivotB(config::ports::pivotBPort, config::ports::endEffectorGearRatio),
      claw(config::ports::clawPort, config::ports::endEffectorGearRatio) {}



void EndEffector::init(){
    pivotA.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    pivotB.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

//closes the claw at a low voltage and zeroes it as soon as it stalls (after the grace time, so it isn't mistaken for a
//stall while it spins up), or at the timeout if it never does. with a preload Pin the stall is on the Pin, so the zero
//is shifted by the Pin grip
//minTravelDeg: a stall only counts once the claw has closed this far. a claw starting open can sit still for a moment
//before it breaks free, which would otherwise read as a stall and zero it while open
void EndEffector::homeClaw(bool preload, double minTravelDeg){
    uint32_t start {pros::millis()};
    double startPosition {claw.get_position()};
    bool stalled {false};
    uint32_t slowSince {0};
    //on a Pin it closes harder, so the stall point is a firm grip that the brake then holds
    claw.move_voltage(-(preload ? cfg::clawClampMv : cfg::clawHomeMv));
    while(true){
        uint32_t elapsed {pros::millis() - start};
        bool slow {elapsed > cfg::clawHomeGraceMs
                   && startPosition - claw.get_position() >= minTravelDeg
                   && std::abs(claw.get_actual_velocity()) < cfg::clawHomeStallRpm};
        //one slow reading mid-close zeroed the claw while it was still closing, so it has to stay slow for 100 ms
        if(!slow){
            slowSince = 0;
        }
        else if(slowSince == 0){
            slowSince = pros::millis();
        }
        stalled = slowSince != 0 && pros::millis() - slowSince >= 100;
        if(stalled || elapsed > cfg::clawHomeTimeoutMs){
            break;
        }
        pros::delay(20);
    }
    std::printf("claw home (%s): %s after %u ms, closed %.1f deg to the zero\n", preload ? "on the Pin" : "empty",
                stalled ? "stalled" : "timed out", static_cast<unsigned>(pros::millis() - start),
                startPosition - claw.get_position());
    claw.brake();
    claw.tare_position();
    clawOffsetDeg = preload ? -cfg::clawPinGripDeg : 0;
}

void EndEffector::rehomeClaw(std::function<bool()> startWhen){
    clawRehoming = true;
    pros::Task([this, startWhen]{
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

void EndEffector::waitForClawHome(){
    while(clawRehoming){
        pros::delay(10);
    }
}

//motors only run once the robot is enabled, so call this at the start of autonomous/opcontrol (alongside the elevator home).
//homes the claw first, then the pivot. the claw closes onto its stop, then the pivot (which starts at its highest
//point) drives up into its top stop, each until it stalls (or times out), then zeroes there. a mechanism that is already
//homed is skipped. with a preload Pin the claw is left alone (clampPreload() homes it on the move)
void EndEffector::forgetHome(){
    clawHomed = false;
    pivotHomed = false;
}

void EndEffector::homeBlocking(bool preload){

    //with a preload the claw home waits until the end (below), so it happens while the robot is already driving
    if(!clawHomed && !preload){
        homeClaw(false);
        clawHomed = true;
    }

    if(preload){
        //push the claw shut on the Pin before the pivot moves, and keep pushing through the pivot home and stow: the
        //brake alone let the Pin fling out. clampPreload() takes over from here, so it stays powered until then
        claw.move_voltage(-cfg::clawClampMv);
    }

    if(!pivotHomed){
        uint32_t start {pros::millis()};
        pivotA.move_voltage(cfg::pivotHomeMv);
        pivotB.move_voltage(cfg::pivotHomeMv);
        while(true){
            uint32_t elapsed {pros::millis() - start};
            //both pivot motors must be slow, so one slipping motor can't end the home early
            double speed {std::max(std::abs(pivotA.get_actual_velocity()), std::abs(pivotB.get_actual_velocity()))};
            bool stalled {elapsed > cfg::pivotHomeGraceMs && speed < cfg::pivotHomeStallRpm};
            if(stalled || elapsed > cfg::pivotHomeTimeoutMs){
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
        pivotTo(cfg::pivotStowDeg);

        //wait for the pivot to reach stow so the elevator (which starts moving once homing returns) doesn't hit it on the way
        uint32_t start {pros::millis()};
        while(std::abs((pivotA.get_position() + pivotB.get_position()) / 2 - cfg::pivotStowDeg) > cfg::pivotStowToleranceDeg
              && pros::millis() - start < cfg::pivotStowTimeoutMs){
            pros::delay(20);
        }
    }
}

void EndEffector::clampPreload(){
    clawRehoming = true;
    clawHomed = true;
    pros::Task([this]{
        homeClaw(true);
        //the brake alone didn't keep the Pin from flinging out when the pivot swings down, so keep pushing the claw shut
        //until the swing is done
        claw.move_voltage(-cfg::clawClampMv);
        pros::delay(config::auton::clawMs);
        pivotTo(cfg::pivotHorizontalDeg);
        pros::delay(config::auton::pivotMs);
        //wiggle the claw open a couple degrees and shut again, so a Pin stuck at an awkward angle settles square. the
        //zero is the Pin grip, so + is open; shutting is the clamp voltage, which the Pin stops
        for(int i {0}; i < cfg::clawWiggles; ++i){
            claw.move_absolute(cfg::clawWiggleDeg, cfg::clawRpm);
            pros::delay(cfg::clawWiggleMs);
            claw.move_voltage(-cfg::clawClampMv);
            pros::delay(cfg::clawWiggleMs);
        }
        claw.brake();
        clawRehoming = false;
    });
}

void EndEffector::openClaw(){
    claw.move_absolute(cfg::clawOpenDeg + clawOffsetDeg, cfg::clawRpm);
}

void EndEffector::closeClaw(){
    claw.move_absolute(clawOffsetDeg, cfg::clawRpm);
}

void EndEffector::pivotTo(double deg){
    pivotA.move_absolute(deg, cfg::pivotRpm);
    pivotB.move_absolute(deg, cfg::pivotRpm);
}

//bench test, call every loop. right stick Y drives both pivots. holding R1 opens the claw, moves the pivot to horizontal and
//the elevator to its intake height; releasing R1 closes the claw.
//if the pivots fight each other, negate pivotBPort in config/ports.h
void EndEffector::test(){
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) != 0){
        openClaw();
        pivotTo(cfg::pivotHorizontalDeg);
        elevator.requestIntake();
    }
    else if(master.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_R1) != 0){
        closeClaw();
    }

    //R2 stows: claw closed, pivot up to the stow position
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2) != 0){
        closeClaw();
        pivotTo(cfg::pivotStowDeg);
    }
}
