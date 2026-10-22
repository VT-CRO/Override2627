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

    void init(){
        pivotA.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        pivotB.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }

    //motors only run once the robot is enabled, so call this at the start of autonomous/opcontrol (alongside the elevator home).
    //homes the claw first, then the pivot. the claw closes onto its stop, then the pivot (which starts at its highest
    //point) drives up into its top stop, each until it stalls, then zeroes there. a mechanism that is already homed is skipped;
    //one that times out is left unhomed (and the next one still runs)
    void homeBlocking(){
        static bool clawHomed {false};
        static bool pivotHomed {false};

        if(!clawHomed){
            std::printf("claw homing started\n");
            uint32_t start {pros::millis()};
            bool moved {false};
            claw.move_voltage(-constants::endEffectorClawHomeMv);
            while(true){
                uint32_t elapsed {pros::millis() - start};
                double speed {std::abs(claw.get_actual_velocity())};
                //a stall only counts after the claw has actually moved, so one that hasn't broken free yet isn't mistaken for the stop
                moved = moved || speed > constants::endEffectorClawHomeMovingRpm;
                if(moved && elapsed > constants::endEffectorClawHomeGraceMs && speed < constants::endEffectorClawHomeStallRpm){
                    std::printf("claw homed after %lu ms\n", static_cast<unsigned long>(elapsed));
                    claw.brake();
                    claw.tare_position();
                    clawHomed = true;
                    break;
                }
                if(elapsed > constants::endEffectorClawHomeTimeoutMs){
                    claw.brake();
                    std::printf("claw home timed out, not zeroed\n");
                    break;
                }
                pros::delay(20);
            }
        }

        if(!pivotHomed){
            std::printf("pivot homing started\n");
            uint32_t start {pros::millis()};
            bool moved {false};
            pivotA.move_voltage(constants::endEffectorPivotHomeMv);
            pivotB.move_voltage(constants::endEffectorPivotHomeMv);
            while(true){
                uint32_t elapsed {pros::millis() - start};
                //both pivot motors must be slow, so one slipping motor can't end the home early
                double speed {std::max(std::abs(pivotA.get_actual_velocity()), std::abs(pivotB.get_actual_velocity()))};
                moved = moved || speed > constants::endEffectorPivotHomeMovingRpm;
                if(moved && elapsed > constants::endEffectorPivotHomeGraceMs && speed < constants::endEffectorPivotHomeStallRpm){
                    std::printf("pivot homed after %lu ms\n", static_cast<unsigned long>(elapsed));
                    pivotA.brake();
                    pivotB.brake();
                    pivotA.tare_position();
                    pivotB.tare_position();
                    pivotHomed = true;
                    break;
                }
                if(elapsed > constants::endEffectorPivotHomeTimeoutMs){
                    pivotA.brake();
                    pivotB.brake();
                    std::printf("pivot home timed out, not zeroed\n");
                    break;
                }
                pros::delay(20);
            }
        }
    }

    //bench test, call every loop. right stick Y drives both pivots. holding R1 opens the claw, moves the pivot to horizontal and
    //the elevator to its intake height; releasing R1 closes the claw. the claw position (0 = closed) and pivot position print twice a second.
    //if the pivots fight each other, negate endEffectorPivotBPort in constants.h
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
            claw.move_absolute(constants::endEffectorClawOpenDeg, constants::endEffectorClawRpm);
            pivotA.move_absolute(constants::endEffectorPivotHorizontalDeg, constants::endEffectorPivotRpm);
            pivotB.move_absolute(constants::endEffectorPivotHorizontalDeg, constants::endEffectorPivotRpm);
            elevator::requestIntake();
        }
        else if(constants::master.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_R1) != 0){
            claw.move_absolute(0, constants::endEffectorClawRpm);
        }

        //R2 stows: claw closed, pivot up to the stow position
        if(constants::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2) != 0){
            claw.move_absolute(0, constants::endEffectorClawRpm);
            pivotA.move_absolute(constants::endEffectorPivotStowDeg, constants::endEffectorPivotRpm);
            pivotB.move_absolute(constants::endEffectorPivotStowDeg, constants::endEffectorPivotRpm);
        }

        static uint32_t lastPrint {0};
        if(pros::millis() - lastPrint >= 500){
            lastPrint = pros::millis();
            std::printf("claw %.1f deg, pivot %.1f deg\n", claw.get_position(), (pivotA.get_position() + pivotB.get_position()) / 2);
        }
    }

}
