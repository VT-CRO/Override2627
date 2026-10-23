#pragma once

namespace elevator{
    void init();

    void homeBlocking();

    //measures the elevator's tuning values and prints them (see elevator_tune.cpp). call every loop, it only runs on LEFT + RIGHT held
    void autoTune();

    //marks the zero as unknown so the next homeBlocking() re-homes
    void forgetHome();

    void update();

    void handleInput();

    void manualCommand();

    void requestStow();

    void requestIntake();

    void requestLift(double rotations);

    //drive to an absolute height (motor rotations above the stow) and hold it, for autonomous
    void requestHeight(double rotations);

    //true once the elevator reached its target and is holding there
    bool isSettled();

    bool isElevatorStowed();

    double getElevatorRotations();
}