#pragma once
#include "api.h"
#include "auton/auton.h"
#include "subsystems/drivetrain.h"
#include "subsystems/elevator.h"
#include "subsystems/endeffector.h"
#include <cstdint>
#include <memory>

//owns the controller and every subsystem, and runs the competition modes. main.cpp only forwards the PROS hooks here
class Robot{
public:
    Robot();

    void init();

    void selectAuton();

    //homes with the preload Pin in the claw, then runs the selected routine
    void runAuton();

    void driverControl();

private:
    //homes the elevator and the claw at the same time; returns once both are done. preload: a Pin starts in the claw
    void homeAll(bool preload);

    //an X + Y test run from driver control: hold X + Y to start the selected auton, B or a stick stops it.
    //true while a run is going (driver control should stand by)
    bool autonTest();

    pros::v5::Controller master;
    Elevator elevator;
    EndEffector endEffector;
    Drivetrain drivetrain;
    Auton auton;

    std::unique_ptr<pros::Task> testTask;
    //RIGHT tap detection for the 90 deg turn test: when it was pressed (0 = up, 1 = pressed with LEFT, never a tap)
    std::uint32_t rightPressedAt {0};
};
