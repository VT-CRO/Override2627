#pragma once

namespace elevator{
    void init();

    void update();

    void handleInput();

    void manualCommand();

    void requestStow();

    bool isElevatorStowed();

    double getElevatorRotations();
}