#pragma once
#include <array>
#include <cstdint>

namespace config::elevator{
    constexpr double maxMotorRotations {6.4};

    //--------  elevator constants  -------- 
    //fraction of maxMotorRotations for each major height: bottom, Y, X, A, top
    constexpr std::array<double, 5> majorIncrements {0.0, 0.25, 0.5, 0.75, 1.0};
    constexpr int stepsPerSection {3}; 

    //intake position: motor rotations above the stowed position where the elevator sits to intake.
    //these are plain motor rotations, not measured heights (the DR4B's height per rotation is not constant)
    constexpr double intakeRotations {1.3};
    //gravity feedforward in millivolts: the voltage that just holds the lift still, at the bottom and at the top, blended
    //by position in between. measure it on the robot (hold it at each end and raise until it stops sagging). 0 = off,
    //the integral term then has to carry the load. a DR4B is normally heaviest to hold low, so expect Low > High. TODO tune
    constexpr double gravityLowMv {1575.0};
    constexpr double gravityHighMv {1500.0};
    //the motor positions those two voltages were measured at, and where the load starts at the bottom: the lowest
    //rotations are slack (unwinding the home tension), where turning the motor lifts nothing, so there is no gravity
    //load to hold. below loadedStartRotations the feedforward is 0, ramping up to the Low value by LowAt.
    //the auto-tune (hold LEFT + RIGHT) measures all three. defaults match its test heights
    constexpr double gravityLowAtRotations {1.92};
    constexpr double gravityHighAtRotations {3.84};
    constexpr double loadedStartRotations {0.88};
    constexpr int returnTimeoutMs {4000};
    //the home (X, or autonomous) stops and zeroes on its own, checked only after homeGraceMs of pushing.
    //if it never reaches the zero tension it zeroes anyway after manualHomeTimeoutMs. TODO tune
    constexpr uint32_t homeGraceMs {300};
    //the chain keeps tightening, so there is no hard stop: the home zeroes at a fixed tension instead.
    //push down at homeMv (millivolts, low = gentle on the chain) until the highest motor current stays above
    //homeStallCurrentMa for the confirm time. TODO tune the current from the "home ... mA" lines printed to the terminal
    constexpr int homeMv {-3500};
    //measured at 3.5 V: free spin ~73 rpm at <60 mA, current plateaus ~1430 mA with velocity 0.0 on the stop
    constexpr int32_t homeStallCurrentMa {1152};
    constexpr double homeStallRpm {34.3};
    constexpr uint32_t homeStallConfirmMs {150};
    constexpr uint32_t manualHomeTimeoutMs {8000};

    //elevator positioning
    //power while holding L1/L2 in manual, -127 to 127 for move()
    constexpr int manualPower {127};
    //manual down (L2) slows as it nears zero so it stops at the stow position instead of coasting below it:
    //within this many rotations of zero the power scales down to the min fraction. TODO tune
    constexpr double manualBottomTaperRotations {0.8};
    constexpr double manualBottomMinScale {0.2};

    //elevator voltage control: one voltage is computed and sent to BOTH motors so they can't fight each other.
    //TODO tune all of these on the robot
    //max voltage the elevator is ever given, in millivolts (12000 = full)
    constexpr double maxVoltageMv {12000.0};
    //P gain: millivolts per rotation of error. 12000 = full power when 1+ rotation away, fades out as it gets close.
    //this sets how fast short moves are: one Up/Down step is only ~0.53 rotations, so a low gain means a low voltage
    constexpr double kp {14682.0};
    //smallest voltage that still moves the elevator (beats friction) so it doesn't stall just short of the target
    constexpr double minVoltageMv {1710.0};
    //voltage slew limits, in millivolts per second. rise = speeding up (48000 = 0 to 12 V in 0.25 s),
    //fall = slowing down (72000 = 12 V to 0 in ~0.17 s). fall must stay fast enough to keep up with the P gain
    //dropping the voltage near the target, or the elevator overshoots. lower = smoother but slower to respond
    constexpr double voltageRiseMvPerSec {48000.0};
    constexpr double voltageFallMvPerSec {72000.0};
    //integral: only builds inside integralZone (rotations) so it can't wind up during big moves, and its voltage
    //is capped at integralMaxMv. it replaces the min voltage inside the zone: it grows until friction/gravity
    //are beaten, so the elevator lands on the target instead of stopping short. TODO tune
    constexpr double ki {237975.0};  //mV per (rotation * second)
    constexpr double integralZone {0.3};
    constexpr double integralMaxMv {3244.0};
    //derivative on measurement: mV per (rotation / second) of speed, brakes the approach so it doesn't overshoot. TODO tune
    constexpr double kd {587.0};
    //settled = within the tolerance AND slower than settleSpeed (rot/s) for settleDwellMs, then brake and hold.
    //requiring it to be slow is what makes the stop height repeatable instead of wherever it first touched the window
    constexpr double settleTolerance {0.02};
    constexpr double settleSpeed {0.05};
    constexpr uint32_t settleDwellMs {100};
    //after settling, only start correcting again if it drifts this far. the gap between the two stops chatter
    constexpr double resettleTolerance {0.05};
}
