#include "drivetrain.h"
#include "constants.h"
#include "api.h"
#include "elevator.h"
#include "lemlib/api.hpp"
#include <cmath>
#include <algorithm>
#include <cstdint>

namespace drivetrain{

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
        .1  //slew rate
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
        .25  //slew rate
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

    //slew state for lemlibArcade: the forward/back voltage (mV) after slewing, and when we last ran
    static double throttleVoltage {0};
    static std::uint32_t lastDriveTime {0};

    //move current toward target, but only by rate * dt. rise rate when the voltage is growing or flipping
    //direction, fall rate when it is shrinking toward 0 (same logic as the elevator's applyVoltage)
    static double slewVoltage(double target, double current, double riseRate, double fallRate, double dtSec){
        bool flipping {(target > 0 && current < 0) || (target < 0 && current > 0)};
        bool growing {std::abs(target) > std::abs(current)};
        double maxStep {((growing || flipping) ? riseRate : fallRate) * dtSec};
        return current + std::clamp(target - current, -maxStep, maxStep);
    }

    void lemlibArcade(){
        double multiplier {1};
        double maxElevatorRotations {constants::elevatorMaxMotorRotations};
        double currentElevatorRotations {elevator::getElevatorRotations()};
        double scalingConstant {1};

        if(currentElevatorRotations > maxElevatorRotations/2){
            scalingConstant = 3.5;
        }
        else{
            scalingConstant = .01;
        }
        

        multiplier = ((constants::elevatorMaxMotorRotations + scalingConstant) -  currentElevatorRotations) / (maxElevatorRotations);

        double leftY {constants::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)};
        double rightX {constants::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)};

        //curve AFTER the multiplier, same order LemLib's arcade used before, so top speeds feel the same as before
        double throttle {throttleCurve.curve(leftY * multiplier)};
        double turn {steerCurve.curve(rightX * multiplier)};

        //time since the last loop. first call or a long gap: assume one 20 ms loop
        std::uint32_t now {pros::millis()};
        double dtSec {(lastDriveTime == 0 || now - lastDriveTime > 100) ? 0.02 : (now - lastDriveTime) / 1000.0};
        lastDriveTime = now;

        //anti tip: the higher the elevator, the gentler the limits (0 = down, 1 = top)
        double height {std::clamp(currentElevatorRotations / maxElevatorRotations, 0.0, 1.0)};
        auto blend {[height](double low, double high){ return low + (high - low) * height; }};

        //stick units (-127 to 127) to millivolts (-12000 to 12000). only translation is slewed, turning goes straight through
        throttleVoltage = slewVoltage(throttle * 12000.0 / 127.0, throttleVoltage,
                                      blend(constants::driveVoltageRiseLowMvPerSec, constants::driveVoltageRiseHighMvPerSec),
                                      blend(constants::driveVoltageFallLowMvPerSec, constants::driveVoltageFallHighMvPerSec), dtSec);
        double turnVoltage {turn * 12000.0 / 127.0};

        //arcade mix into left/right, scaling both down together if one side goes past 12 V (keeps the turn ratio)
        double leftVoltage {throttleVoltage + turnVoltage};
        double rightVoltage {throttleVoltage - turnVoltage};
        double biggest {std::max(std::abs(leftVoltage), std::abs(rightVoltage))};
        if(biggest > 12000){
            leftVoltage *= 12000 / biggest;
            rightVoltage *= 12000 / biggest;
        }

        //straight to the motors (LemLib odometry still tracks the robot, it reads the encoders on its own)
        leftMotorGroup.move_voltage(static_cast<int>(leftVoltage));
        rightMotorGroup.move_voltage(static_cast<int>(rightVoltage));
    }

    void drive(){
        lemlibArcade();
        }

    void init(){
        leftMotorGroup.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
        rightMotorGroup.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
        chassis.calibrate();
    }
}