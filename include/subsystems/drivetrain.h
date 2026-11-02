#pragma once
#include "api.h"
#include "lemlib/api.hpp"
#include "subsystems/elevator.h"
#include <cmath>
#include <cstdint>

class Drivetrain{
public:
    Drivetrain(pros::v5::Controller& master, const Elevator& elevator);

    void init();

    //driver arcade control, call every loop. hands the motors to a running LemLib motion until a stick moves
    void drive();

    //turns in place by degrees (clockwise positive) with the angular PID, prints the end error and overshoot.
    //returns right away, the turn runs in its own task
    void turnBy(double degrees);

    //drives ahead at power (0-127), steering to hold heading (deg), until both sides' motors stall (see
    //config::auton::pushStallRpm) or timeoutMs, then brakes. blocks
    void pushUntilStall(int power, std::uint32_t timeoutMs, double heading);

    //measures the turning model and prints angularController gains (see tune/drivetrain_tune.cpp). call every loop, it only runs on RIGHT held alone
    void rotationSysId();

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

public:
    //the LemLib chassis, for autonomous motions. declared last: it is built from the members above
    lemlib::Chassis chassis;
};
