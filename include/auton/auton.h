#pragma once
#include "api.h"
#include "lemlib/api.hpp"
#include "subsystems/drivetrain.h"
#include "subsystems/elevator.h"
#include "subsystems/endeffector.h"
#include <array>
#include <atomic>
#include <cstdint>
#include <functional>

class Auton{
public:
    Auton(pros::v5::Controller& master, Drivetrain& drivetrain, Elevator& elevator, EndEffector& endEffector);

    //controller selector: LEFT/RIGHT cycle the routines, the choice shows on the controller screen. loops until enabled
    void select();

    //runs the selected routine and keeps the elevator updating while it does
    void run();

    //true if the selected routine homes the elevator itself, while it drives, instead of before it starts
    bool homesElevatorWhileMoving() const;

    //true while an X + Y test run from driver control is going (no competition switch to say autonomous is on)
    std::atomic<bool> testing {false};

private:
    struct Routine{
        const char* name;
        void (Auton::*run)();
    };

    double facePoint(double x, double y, int maxSpeed = 127);
    void waitElevatorNear(double rotations, double toleranceRotations, std::uint32_t waitMs = 1000);
    void release(int layer, double openExtraDeg = 10.8);
    //extraBackoffIn: backs off this much more before the drop. pushSpeed (0-127): push in at this power instead of
    //config pushSpeed (0 = use the config one). openExtraDeg: opens the claw this many degrees past the normal open
    //position to drop the Pin
    void score(int layer, double heading, double extraBackoffIn = 0, int pushSpeed = 0, double openExtraDeg = 10.8);
    void backStraight(double inches, const lemlib::MoveToPointParams& params, int timeoutOverrideMs = 0);
    std::function<bool()> movedFromHere(double inches);
    void intakeWhenClear();
    void liftWhenClear(double rotations, double inches);
    void driveSlowEnd(double dx, double dy, double slowIn, int slowSpeed, int fastSpeed, int slowTimeoutMs);
    void pickUp(double x, double y, double liftTo, bool clawFirst = false, double closeExtraDeg = 0);
    void skills();
    void showPose();
    void drive48();
    void turn90();
    void ogpreload();
    void matchLeft();
    void matchRight();
    void none();

    static const std::array<Routine, 7> routines;

    pros::v5::Controller& master;
    Drivetrain& drivetrain;
    lemlib::Chassis& chassis;
    Elevator& elevator;
    EndEffector& endEffector;

    int selected {0};
    //how far the last score() moved the origin: a point in the old frame plus this is the same point in the new frame
    double scoreShiftX {0};
    double scoreShiftY {0};
    //true while the claw is re-homing after the preload and hasn't opened again yet
    std::atomic<bool> clawReopening {false};
};
