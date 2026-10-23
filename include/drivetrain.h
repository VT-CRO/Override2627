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

    //autonomous motions on a trapezoidal profile (see the auton constants). both block until done.
    //drives straight along the current heading (negative = backwards), holding that heading
    //speedScale multiplies the profile's max speed and acceleration (1 = the tuned, accurate speed). slowEndIn > 0 makes the
    //last slowEndIn inches run at the tuned speed, reached by a smooth slow-down from the faster cruise (no stop in between)
    void driveStraight(double inches, double speedScale = 1.0, double slowEndIn = 0.0);

    //drives straight ahead for `inches` like driveStraight but only slows to the base speed (not to a stop), then carries
    //straight on into something (the goal's aligner) at that speed's power for pushMs and brakes. blocks until done
    //holdHeading (degrees, NaN = keep the starting heading) is what it steers toward on the way in
    void driveInto(double inches, std::uint32_t pushMs, double speedScale = 1.0, double holdHeading = NAN);

    //turns in place the short way to a field heading (degrees, 0 = +y, clockwise positive)
    //toleranceDeg > 0 lets the turn end once within that many degrees (default: the angular controller's 1 deg)
    void turnTo(double heading, double speedScale = 1.0, double toleranceDeg = 0.0);

    //temporary: prints the odometry position and heading (what the moves steer by) and the IMU's own unscaled reading to
    //the terminal twice a second, for checking odometry against the field. call every loop
    void printHeading();

    //measures the turning model and prints angularController gains (see drivetrain_tune.cpp). call every loop, it only runs on RIGHT held alone
    void rotationSysId();
}