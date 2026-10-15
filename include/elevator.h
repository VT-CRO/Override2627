#pragma once

namespace elevator{
    void init();

    void requestHome();

    void homeBlocking();

    bool isHoming();

    void update();

    void handleInput();

    void manualCommand();

    void requestStow();

    void requestScoreEnd();

    void requestLift(double inches);

    bool isElevatorStowed();

    double getElevatorRotations();
}