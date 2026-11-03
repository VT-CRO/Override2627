#pragma once
#include "api.h"
#include "config/drivetrain.h"
#include "lemlib/api.hpp"
#include "subsystems/elevator.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <vector>
#include <memory>
#include <atomic>

class Drivetrain{
public:
    Drivetrain(pros::v5::Controller& master, const Elevator& elevator);

    void init();

    //driver arcade control, call every loop. hands the motors to a running LemLib motion until a stick moves
    void drive();

    //turns in place by degrees (clockwise positive) with turnToHeading, prints the end error and overshoot.
    //returns right away, the turn runs in its own task
    void turnBy(double degrees);

    //turns in place to heading (deg, LemLib's frame, the short way round) with TurnController, which follows the
    //elevator's height (see util/turn_controller.h). it joins LemLib's motion queue, so chassis.isInMotion() and
    //cancelMotion() work on it. exitRangeDeg > 0 hands over once that close without braking, for a move that follows.
    //speedFraction (0-1] slows just this turn. blocks unless async. `started` is only for the async hand-off to itself
    void turnToHeading(double heading, std::uint32_t timeoutMs, double exitRangeDeg = 0, double speedFraction = 1,
                       bool async = false, std::shared_ptr<std::atomic<bool>> started = nullptr);

    //turnToHeading toward the point (x, y), inches in LemLib's frame
    void turnToPoint(double x, double y, std::uint32_t timeoutMs, double exitRangeDeg = 0, double speedFraction = 1,
                     bool async = false);

    //drives ahead at power (0-127), steering to hold heading (deg), until both sides' motors stall (see
    //config::auton::pushStallRpm) or timeoutMs, then brakes. blocks
    void pushUntilStall(int power, std::uint32_t timeoutMs, double heading);

    //measures the turning model at each elevator height and prints it for config/drivetrain.h (see tune/drivetrain_tune.cpp).
    //call every loop, it only runs on RIGHT held alone. holdElevator(fraction of travel) is called every 10 ms while it
    //runs and must drive the elevator there
    void rotationSysId(const std::function<void(double)>& holdElevator);

private:
    //the IMU read 3635.2 deg over a true 3600 deg spin, so scale its angles. LemLib reads its heading through these
    class ScaledImu : public pros::Imu{
        static constexpr double scale {3600.0 / 3635.2};
    public:
        using pros::Imu::Imu;

        double get_rotation() const override {
            return pros::Imu::get_rotation() * scale;
        }

        double get_heading() const override {
            double heading {std::fmod(get_rotation(), 360.0)};
            return heading < 0 ? heading + 360.0 : heading;
        }

        //turn rate in deg/s, clockwise positive like the heading, scaled the same way
        double turnRate() const {
            return pros::Imu::get_gyro_rate().z * scale * config::drivetrain::imuRateSign;
        }
    };

    pros::v5::Controller& master;
    const Elevator& elevator;
    pros::MotorGroup leftMotors;
    pros::MotorGroup rightMotors;
    ScaledImu imu;
    pros::adi::Encoder vertEncoder;
    pros::adi::Encoder horizEncoder;
    lemlib::TrackingWheel vertPod;
    lemlib::TrackingWheel horizPod;
    lemlib::ExpoDriveCurve throttleCurve;
    lemlib::ExpoDriveCurve steerCurve;

    //slew state for drive(): the forward/back voltage (mV) after slewing, and when we last ran
    double throttleVoltage {0};
    double turnVoltage {0};
    std::uint32_t lastDriveTime {0};

    //heading hold for drive(): with no turn stick, steer to holdHeading (deg) so the robot drives dead straight
    bool holdingHeading {false};
    double holdHeading {0};
    double lastTheta {0};
    std::uint32_t turnReleasedAt {0};

    //the last turnToHeading, one row per loop: ms since start, elevator height (0-1), heading, gyro rate, heading
    //difference rate (deg/s), plan acceleration (deg/s^2), output (mV). turnBy prints it, for checking a turn by eye
    std::vector<std::array<float, 7>> turnLog;

public:
    //lemlib::Chassis with its motion queue opened up, so turnToHeading queues and cancels like LemLib's own motions
    class MotionChassis : public lemlib::Chassis{
    public:
        using lemlib::Chassis::Chassis;
        using lemlib::Chassis::requestMotionStart;
        using lemlib::Chassis::endMotion;

        //false once the motion was cancelled
        bool motionActive() const {
            return motionRunning;
        }
    };

    //the LemLib chassis, for autonomous motions. declared last: it is built from the members above
    MotionChassis chassis;
};
