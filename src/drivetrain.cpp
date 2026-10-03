#include "drivetrain.h"
#include "constants.h"
#include "api.h"
#include "lemlib/api.hpp"
#include <cmath>

namespace drivetrain{

        enum class Mode {tank, customArcade, lemlibArcade};

    static Mode currentMode {Mode::lemlibArcade};

    static pros::MotorGroup leftMotorGroup(
        {constants::leftMotorOne,
        constants::leftMotorTwo,
        constants::leftMotorThree},
        constants::drivetrainGearRatio);

    static pros::MotorGroup rightMotorGroup(
        {constants::rightMotorOne, 
        constants::rightMotorTwo, 
        constants::rightMotorThree}, 
        constants::drivetrainGearRatio);

    pros::Imu imu(constants::imu);

    //read documentation to understand each param
    //TODO tune to real bot 
    static lemlib::Drivetrain drivetrain(
        &leftMotorGroup, 
        &rightMotorGroup, 
        10, 
        lemlib::Omniwheel::NEW_325,
        200, 
        2
    );

    //TODO tune
    static lemlib::ControllerSettings lateralController(
        5,  //P
        0,  //I
        8,  //D
        0,
        0,
        0,
        0,
        0,
        15  //slew rate
    );

    //TODO tune
    static lemlib::ControllerSettings angularController(
        5,  //P
        0,  //I
        0,  //D
        0,
        0,
        0,
        0,
        0,
        3  //slew rate
    );

    //TODO tune
    static lemlib::OdomSensors sensors(
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        &imu
    );

    //TODO tune
    lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
    lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

    static lemlib::Chassis chassis(
        drivetrain,
        lateralController,
        angularController,
        sensors,
        &throttleCurve,
        &steerCurve
    );

    void tankDrive(){
        //left side of bot
        leftMotorGroup.move(constants::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));

        //right side of bot
        rightMotorGroup.move(constants::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
    }

    //blend movements into two sticks
    void customArcadeDrive(){
        //initalize as one to keep value same if within bounds of [-127, 127]
        double multiplier {1};
        
        //take in the stick vlaue for each stick and blend them
        //not using brace initalization b/c of warnings thrown
        double leftSum = constants::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + constants::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        //left is y + x
        double rightSum = constants::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) - constants::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        //right is y - x

        //handels wrapping by dividing by same multiple to maintin users intended input for each movement 
        if(leftSum > 127 || leftSum < -127){
            multiplier = std::abs(127/leftSum);
        }
        else if(rightSum > 127 || rightSum < -127){
            multiplier = std::abs(127/rightSum);
        }

        //takes the sum of the cordinates for each sides of the stick and handels overflow by keeping ratio and maxing each side out at [-127, 127]
        leftMotorGroup.move(std::round(leftSum * multiplier));

        rightMotorGroup.move(std::round(rightSum * multiplier));
    }

    void userSwitchingModes(){
        if(constants::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
            switch(currentMode){
                case Mode::tank:
                currentMode = Mode::customArcade;
                constants::master.rumble(".");
                break;

                case Mode::customArcade:
                constants::master.rumble(".");
                currentMode = Mode::lemlibArcade;
                break;

                case Mode::lemlibArcade:
                constants::master.rumble(".");
                currentMode = Mode::tank;
                break;
            }  
        }
    }

    void lemlibArcade(){
        int leftY {constants::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)};
        int rightX {constants::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)};

        chassis.arcade(leftY, rightX);
    }

    void drive(){
        switch(currentMode){
            case Mode::tank:
            tankDrive();
            break;

            case Mode::customArcade:
            customArcadeDrive();
            break;

            case Mode::lemlibArcade:
            lemlibArcade();
            break;
        }
    }

    void init(){
        leftMotorGroup.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
        rightMotorGroup.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
        chassis.calibrate();
    }
}