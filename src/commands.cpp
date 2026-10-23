#include "elevator.h"
#include "drivetrain.h"
#include "constants.h"
#include "api.h"
#include "lemlib/api.hpp"
#include <cmath>
#include <cstdint>

namespace commands{

    //R2: lower the elevator to the intake height
    void score(){
        if(constants::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2) == 1){
            elevator::requestIntake();
        }
    }

    //LEFT: drive forward one meter. LEFT is otherwise only used with RIGHT for the elevator auto-tune, so ignore it while RIGHT is held
    void driveForwardMeter(){
        if(constants::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT) == 1
           && constants::master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) == 0){
            drivetrain::driveForward(100.0 / 2.54);
        }
    }

    //RIGHT tap: turn 90 deg clockwise to check the angular gains. fires on release, since holding RIGHT 1.5 s is the
    //rotation sysID (which blocks the loop while held, so new_press never sees the press). call before rotationSysId().
    //ignored when LEFT is down, LEFT + RIGHT is the elevator auto-tune
    void turnNinety(){
        static std::uint32_t pressedAt {0};
        bool right {constants::master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) != 0};
        bool left {constants::master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) != 0};
        if(right && pressedAt == 0){
            pressedAt = left ? 1 : pros::millis(); //1 = pressed with LEFT, never a tap
        }
        else if(!right && pressedAt != 0){
            if(pressedAt != 1 && !left && pros::millis() - pressedAt < 1500){
                drivetrain::turnBy(90.0);
            }
            pressedAt = 0;
        }
    }

}
