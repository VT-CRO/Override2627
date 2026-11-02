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

    //true while an X + Y test run from driver control is going (no competition switch to say autonomous is on)
    std::atomic<bool> testing {false};

private:
    struct Routine{
        const char* name;
        void (Auton::*run)();
    };

    void watchTurn(const char* label, double target);
    double facePoint(const char* label, double x, double y, float maxSpeed = 127);
    void waitElevatorNear(double rotations, double toleranceRotations, std::uint32_t timeoutMs = 1000);
    void release(int layer);
    void score(int layer, double heading);
    void backStraight(double inches, const lemlib::MoveToPointParams& params);
    std::function<bool()> movedFromHere(double inches);
    void intakeWhenClear();
    void pickUp(const char* label, double x, double y, double liftTo, bool clawFirst = false);
    void preloadTest();
    void matchLeft();
    void matchRight();
    void none();

    static const std::array<Routine, 4> routines;

    pros::v5::Controller& master;
    Drivetrain& drivetrain;
    lemlib::Chassis& chassis;
    Elevator& elevator;
    EndEffector& endEffector;

    int selected {0};
    //true while the claw is re-homing after the preload and hasn't opened again yet
    std::atomic<bool> clawReopening {false};
};
