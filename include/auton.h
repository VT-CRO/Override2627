#pragma once

namespace auton{
    //controller selector: LEFT/RIGHT cycle the routines, the choice shows on the controller screen. loops until enabled
    void select();

    //runs the selected routine (skills by default)
    void run();

    //runs the selected auton from driver control, for testing without a competition switch: hold X + Y to start it,
    //press B or move a stick to stop it. call every loop; true while a run is going (driver control should stand by)
    bool driverTest();}
