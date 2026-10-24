#include "drivetrain.h"
#include "constants.h"
#include "api.h"
#include "elevator.h"
#include "lemlib/api.hpp"
#include <cmath>
#include <algorithm>
#include <cstdint>

namespace drivetrain
{

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

    // read documentation to understand each param
    // TODO tune to real bot
    static lemlib::Drivetrain drivetrain(
        &leftMotorGroup,
        &rightMotorGroup,
        10,
        lemlib::Omniwheel::NEW_325,
        200,
        2);

    // TODO tune
    static lemlib::ControllerSettings lateralController(
        5, // P
        0, // I
        8, // D
        0,
        0,
        0,
        0,
        0,
        .1 // slew rate
    );

    // TODO tune
    static lemlib::ControllerSettings angularController(
        5, // P
        0, // I
        0, // D
        0,
        0,
        0,
        0,
        0,
        .25 // slew rate
    );

    // TODO tune
    static lemlib::OdomSensors sensors(
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        &imu);

    // TODO tune
    lemlib::ExpoDriveCurve throttleCurve(3, constants::driveMinOutput, 1.019);
    lemlib::ExpoDriveCurve steerCurve(3, constants::driveMinOutput, 1.019);

    static lemlib::Chassis chassis(
        drivetrain,
        lateralController,
        angularController,
        sensors,
        &throttleCurve,
        &steerCurve);

    // slew state for lemlibArcade: the forward/back voltage (mV) after slewing, and when we last ran
    static double throttleVoltage{0};
    static double turnVoltage{0};
    static std::uint32_t lastDriveTime{0};

    // move current toward target, but only by rate * dt. rise rate when the voltage is growing or flipping
    // direction, fall rate when it is shrinking toward 0 (same logic as the elevator's applyVoltage)
    static double slewVoltage(double target, double current, double riseRate, double fallRate, double dtSec)
    {
        bool flipping{(target > 0 && current < 0) || (target < 0 && current > 0)};
        bool growing{std::abs(target) > std::abs(current)};
        double maxStep{((growing || flipping) ? riseRate : fallRate) * dtSec};
        return current + std::clamp(target - current, -maxStep, maxStep);
    }

    void lemlibArcade()
    {
        double maxElevatorRotations{constants::elevatorMaxMotorRotations};
        double currentElevatorRotations{elevator::getElevatorRotations()};

        double leftY{constants::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)};
        double rightX{constants::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)};

        // anti tip: the higher the elevator, the gentler the limits (0 = down, 1 = top)
        double height{std::clamp(currentElevatorRotations / maxElevatorRotations, 0.0, 1.0)};

        // top speed fades linearly from full (down) to elevatorTopSpeedScale (top), applied after the curve
        double speedScale{1.0 - (1.0 - constants::elevatorTopSpeedScale) * height};
        // only the part above the friction minimum is scaled, so the minimum voltage jump survives
        auto scaleAboveMin{[](double v, double scale)
                           {
                               return v == 0 ? 0.0 : std::copysign(constants::driveMinOutput + (std::abs(v) - constants::driveMinOutput) * scale, v);
                           }};
        double throttle{scaleAboveMin(throttleCurve.curve(leftY), speedScale)};
        double turn{scaleAboveMin(steerCurve.curve(rightX), speedScale * constants::driveTurnScale)};

        // time since the last loop. first call or a long gap: assume one 20 ms loop
        std::uint32_t now{pros::millis()};
        double dtSec{(lastDriveTime == 0 || now - lastDriveTime > 100) ? 0.02 : (now - lastDriveTime) / 1000.0};
        lastDriveTime = now;

        auto blend{[height](double low, double high)
                   { return low + (high - low) * height; }};

        // stick units (-127 to 127) to millivolts (-12000 to 12000). both translation and turning are slewed
        throttleVoltage = slewVoltage(throttle * 12000.0 / 127.0, throttleVoltage,
                                      blend(constants::driveVoltageRiseLowMvPerSec, constants::driveVoltageRiseHighMvPerSec),
                                      blend(constants::driveVoltageFallLowMvPerSec, constants::driveVoltageFallHighMvPerSec), dtSec);
        turnVoltage = slewVoltage(turn * 12000.0 / 127.0, turnVoltage,
                                  constants::driveTurnRiseMvPerSec, constants::driveTurnFallMvPerSec, dtSec);

        // arcade mix into left/right, scaling both down together if one side goes past 12 V (keeps the turn ratio)
        double leftVoltage{throttleVoltage + turnVoltage};
        double rightVoltage{throttleVoltage - turnVoltage};
        double biggest{std::max(std::abs(leftVoltage), std::abs(rightVoltage))};
        if (biggest > 12000)
        {
            leftVoltage *= 12000 / biggest;
            rightVoltage *= 12000 / biggest;
        }

        // straight to the motors (LemLib odometry still tracks the robot, it reads the encoders on its own)
        // move_voltage ignores the brake mode, so brake() only once the slew has fully ramped down to 0
        if (throttleVoltage == 0 && turnVoltage == 0)
        {
            leftMotorGroup.brake();
            rightMotorGroup.brake();
        }
        else
        {
            leftMotorGroup.move_voltage(static_cast<int>(leftVoltage));
            rightMotorGroup.move_voltage(static_cast<int>(rightVoltage));
        }
    }

    void drive()
    {
        lemlibArcade();
    }

    void init()
    {
        leftMotorGroup.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
        rightMotorGroup.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
        chassis.calibrate();
    }
}
