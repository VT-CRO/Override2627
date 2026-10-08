#pragma once

namespace endeffector{
    void init();

    void update();

    void handleInputs();

    void requestStow();

    void requestPickUp();

    void requestTop();

    bool driverOverrode();

    void clearDriverOverride();
}