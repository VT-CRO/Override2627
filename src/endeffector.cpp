#include "constants.h"
#include "api.h"
#include "endeffector.h"
#include "lemlib/api.hpp"
#include <cmath>
#include <algorithm>

namespace endeffector{
    pros::Motor endEffector(constants::leftEndEffectorMotor, constants::endEffectorGears);

    static uint32_t homingStartTime {0};
    static bool homed {false};

    enum class Mode{Homing, Stowed, PickUp, High, Top};

    Mode currentMode {Mode::Homing};

    //angles are 0 = down, 90 = PickUp, but the encoder is zeroed at the top hard stop (180),
    //so convert a prong angle to a motor position before sending it to move_absolute
    //also clamps the angle to [0, top setpoint] first: there is no bottom stop, so a bad target
    //could otherwise swing the prongs past 0 into the robot
    static double toMotorDeg(double angle){
        double safeAngle {std::clamp(angle, static_cast<double>(constants::endEffectorBottomAngle),
                                            static_cast<double>(constants::endEffectorTopDeg))};
        return safeAngle - constants::endEffectorHardStopDeg;
    }

    void init(){
    endEffector.set_encoder_units_all(pros::MotorUnits::degrees);
    endEffector.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    endEffector.brake();
    }

    void update(){
        switch(currentMode){
            case(Mode::Homing): {

            if(homingStartTime == 0){
                homingStartTime = pros::millis();
            }

            uint32_t elapsed {pros::millis() - homingStartTime};

            if(elapsed < constants::endEffectorHomingTimeMs){
                    //pushing up into the top hard stop
                    endEffector.move(constants::endEffectorHomingPower);
                }
                else{
                    //pushed up long enough to be sitting on the top stop, zero here (0 = 180 deg)
                    endEffector.move(0);
                    endEffector.tare_position_all();
                    homed = true;
                    currentMode = Mode::PickUp;
                }
                break;
            }
            
            case(Mode::Stowed):
            endEffector.move_absolute(toMotorDeg(constants::endEffectorBottomAngle), constants::endEffectorMoveVelocity);
            break;

            case(Mode::PickUp):
            endEffector.move_absolute(toMotorDeg(constants::endEffectorHorizonatalDeg), constants::endEffectorMoveVelocity);
            break;

            case(Mode::High):
            endEffector.move_absolute(toMotorDeg(constants::endEffectorTopDeg), constants::endEffectorMoveVelocity);
            break;

            case(Mode::Top):
            //the encoder is zeroed at the top hard stop, so 0 is the stop itself.
            //skips toMotorDeg on purpose: its clamp would cap this at endEffectorTopDeg
            endEffector.move_absolute(0, constants::endEffectorMoveVelocity);
            break;
        }
    }
    void handleInputs(){
        bool LBPressed {constants::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1) != 0};
        bool RBPressed {constants::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) != 0};

        switch(currentMode){
            case(Mode::Homing):
            break;   
            
            case(Mode::Stowed):
            if(LBPressed){
                break;
            }
            else if(RBPressed){
                currentMode = Mode::PickUp;
                break;
            }
            break;
            
            case(Mode::PickUp):
            if(LBPressed){
                currentMode = Mode::Stowed;
            }
            else if(RBPressed){
                currentMode = Mode::High;
                break;
            }
            break;

            case(Mode::High):
            if(LBPressed){
                currentMode = Mode::PickUp;
                break;
            }
            else if(RBPressed){
                break;
            }
            break;

            case(Mode::Top):
            //only the score command sends the prongs here, so R1/L1 are ignored until it sends them back to PickUp
            break;
        }
    }

    //helpers for commands 
    void requestStow(){
        currentMode = Mode::Stowed;
    }

    void requestTop(){
        currentMode = Mode::Top;
    }

    void requestPickUp(){
        currentMode = Mode::PickUp;
    }
}