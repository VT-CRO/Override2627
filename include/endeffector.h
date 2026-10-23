#pragma once
#include <functional>

namespace endeffector{
    void init();

    //preload: a Pin is already in the claw (autonomous), so only the pivot homes; the claw is left for clampPreload()
    void homeBlocking(bool preload = false);

    //closes the claw onto the preload Pin and zeroes it there, then swings the pivot down to horizontal, in the
    //background (so call it as the robot starts moving). returns right away; waitForClawHome() before the claw's next move
    void clampPreload();

    //marks the claw and pivot as unhomed so the next homeBlocking() homes them again
    void forgetHome();

    void test();

    //claw and pivot moves for autonomous. they return right away, the motors finish the move on their own
    void openClaw();

    void closeClaw();

    //both pivot motors to this angle (0 = top, endEffectorPivotHorizontalDeg = scoring)
    void pivotTo(double deg);

    //homes the claw again in the background, with it empty and open (e.g. right after dropping the preload, whose home
    //was shifted by the Pin grip). it starts once startWhen() is true (or after 3 s). returns right away; call
    //waitForClawHome() before moving the claw again
    void rehomeClaw(std::function<bool()> startWhen = []{ return true; });

    //blocks until a rehomeClaw() in progress is done (returns at once if none is)
    void waitForClawHome();}
