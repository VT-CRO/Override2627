#pragma once
#include "api.h"
#include <cstdint>

class Elevator{
public:
    explicit Elevator(pros::v5::Controller& master);

    void init();

    //motors only run once the robot is enabled. blocks until the home finishes
    void homeBlocking();

    //marks the zero as unknown so the next homeBlocking() re-homes
    void forgetHome();

    //runs the state machine, call every loop
    void update();

    void handleInput();

    //safe to call every loop: does nothing if it is already on its way
    void requestIntake();

    //drive to an absolute height (motor rotations above the stow) and hold it, for autonomous
    void requestHeight(double rotations);

    double rotations() const;

    //measures the elevator's tuning values and prints them (see tune/elevator_tune.cpp). call every loop, it only runs on LEFT + RIGHT held
    void autoTune();

private:
    //Stowed: also the startup mode, so nothing moves until the elevator is homed (X in driver control, or autonomous)
    //Positioning: driving to the current level
    //Returning: driving to encoder zero with position control (B tap)
    //ManualHoming: the home (homeBlocking), driving down until it stalls on the bottom stop, then zeroing
    //Manual: driving up/down for as long as the driver holds L1 (up) or L2 (down)
    //ManualHold: L1/L2 released, holding wherever manual stopped
    enum class Mode { Stowed, Positioning, Intake, Lift, Returning, ManualHoming, Manual, ManualHold };

    void applyVoltage(double targetMv);
    void brakeNow();
    void driveToTarget(double target);
    bool manualPastLimit(double voltage) const;
    void stow();
    void goToLevel(int newLevel);

    pros::v5::Controller& master;
    pros::MotorGroup motors;

    double position {0};
    Mode mode {Mode::Stowed};
    int level {0};
    bool homed {false};
    //target (rotations) for Mode::Lift
    double liftTarget {0};

    uint32_t returnStartTime {0};
    //home stall detection: when it started, and when the elevator first looked stalled (0 = moving)
    uint32_t manualHomeStartTime {0};
    uint32_t manualHomeStallSince {0};
    //power used by Manual, set by handleInput (positive = up)
    int manualPower {0};

    //voltage control state: the voltage we last sent (mV, slew limited), when we last sent it,
    //the target we are driving to, and whether we reached it and are braking
    double appliedVoltage {0};
    uint32_t lastVoltageTime {0};
    double controlTarget {0};
    bool holdSettled {false};
    //PID state: integral term (rotation * second), when the elevator first looked settled (0 = not), and measured speed (rot/s)
    double integral {0};
    uint32_t settleSince {0};
    double velocity {0};
    double lastPosition {0};
    uint32_t lastPositionTime {0};
};
