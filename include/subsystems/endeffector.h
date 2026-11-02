#pragma once
#include "api.h"
#include "subsystems/elevator.h"
#include <functional>

class EndEffector{
public:
    EndEffector(pros::v5::Controller& master, Elevator& elevator);

    void init();

    //preload: a Pin is already in the claw (autonomous), so only the pivot homes; the claw is left for clampPreload()
    void homeBlocking(bool preload = false);

    //closes the claw onto the preload Pin and zeroes it there, then swings the pivot down to horizontal, in the
    //background (so call it as the robot starts moving). returns right away; waitForClawHome() before the claw's next move
    void clampPreload();

    //marks the claw and pivot as unhomed so the next homeBlocking() homes them again
    void forgetHome();

    //bench test, call every loop
    void test();

    //claw and pivot moves for autonomous. they return right away, the motors finish the move on their own
    void openClaw();

    void closeClaw();

    //both pivot motors to this angle (0 = top, config::endeffector::pivotHorizontalDeg = scoring)
    void pivotTo(double deg);

    //homes the claw again in the background, with it empty and open (e.g. right after dropping the preload, whose home
    //was shifted by the Pin grip). it starts once startWhen() is true (or after 3 s). returns right away; call
    //waitForClawHome() before moving the claw again
    void rehomeClaw(std::function<bool()> startWhen = []{ return true; });

    //blocks until a rehomeClaw() in progress is done (returns at once if none is)
    void waitForClawHome();

private:
    void homeClaw(bool preload, double minTravelDeg = 0);

    pros::v5::Controller& master;
    Elevator& elevator;
    pros::Motor pivotA;
    pros::Motor pivotB;
    pros::Motor claw;

    //a preload home zeroes the claw where it stalls on the Pin, not at its stop. this shifts every claw target so the rest
    //of the code keeps using the normal degrees. 0 after an empty home
    double clawOffsetDeg {0};
    bool clawRehoming {false};
    bool clawHomed {false};
    bool pivotHomed {false};
    //bench test: only take over the pivot from a move_absolute while the stick is used, or when it was just released
    bool stickActive {false};
};
