#pragma once
#include "lemlib/api.hpp"
#include <cmath>

namespace drivetrain{
    //the LemLib chassis, for autonomous motions
    extern lemlib::Chassis chassis;

    void userSwitchingModes();

    void drive();

    void init();

    //drives straight ahead along the current heading using odometry. returns right away, drive() hands the motors
    //over until it finishes (or a stick moves, which cancels it)
    void driveForward(double inches);

    //turns in place by degrees (clockwise positive) with the angular PID, prints the end error and overshoot.
    //returns right away like driveForward
    void turnBy(double degrees);

    //drives ahead at power (0-127), steering to hold heading (deg), until both sides' motors stall (see
    //autonPushStallRpm) or timeoutMs, then brakes. blocks
    void pushUntilStall(int power, std::uint32_t timeoutMs, double heading);

    //temporary: prints the odometry position and heading (what the moves steer by) and the IMU's own unscaled reading to
    //the terminal twice a second, for checking odometry against the field. call every loop
    void printHeading();

    //measures the turning model and prints angularController gains (see drivetrain_tune.cpp). call every loop, it only runs on RIGHT held alone
    void rotationSysId();
}