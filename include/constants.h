#pragma once
#include "api.h"
#include "lemlib/api.hpp"
#include <array>
#include <cstdint>

namespace constants{
    //motor ports negative for init reverse motion
    
    //drivetrain motors
    constexpr int rightMotorOne {11}; 
    constexpr int rightMotorTwo {-12};
    constexpr int rightMotorThree {13};
    constexpr int leftMotorOne {-1};
    constexpr int leftMotorTwo {2};
    constexpr int leftMotorThree {-3};

    //elevator motors
    constexpr int leftElevator {5};
    constexpr int rightElevator {-19};
    constexpr double elevatorMaxMotorRotations {6.4};

    //end effector (two pivot motors + claw motor)
    constexpr int endEffectorPivotAPort {8};
    constexpr int endEffectorPivotBPort {-6}; //negate if it spins opposite pivot A
    constexpr int endEffectorClawPort {10};

    //--------  drivetrain constants  -------- 
    //driver control acceleration limits, same idea as the elevator: the voltage may only change so fast.
    //only forward/back (translation) is limited, turning (rotation) is not, then the two are mixed into left/right.
    //in millivolts per second (12000 mV = full). rise = speeding up, fall = slowing down (lowering the voltage
    //on a spinning motor brakes it). blended from Low (elevator down) to High (elevator at top). lower = gentler. TODO tune
    //translation (~17% gentler than the first tune)
    constexpr double driveVoltageRiseLowMvPerSec {25000.0};   //elevator down: 0 to 12 V in 0.48 s
    constexpr double driveVoltageFallLowMvPerSec {25000.0};
    constexpr double driveVoltageRiseHighMvPerSec {7500.0};   //elevator up: 0 to 12 V in 1.6 s
    constexpr double driveVoltageFallHighMvPerSec {7500.0};
    //turning: stick scale and slew (fall faster than rise so it stops quickly). TODO tune with a driver
    //smallest stick output (past the deadband) that still overcomes friction; the speed scaling never shrinks it
    constexpr double driveMinOutput {10.0};
    constexpr double driveTurnScale {0.7};
    constexpr double driveTurnRiseMvPerSec {40000.0};
    constexpr double driveTurnFallMvPerSec {60000.0};
    //top speed (fraction of full) with the elevator at the top, fading linearly from 1.0 with the elevator down. TODO tune
    constexpr double elevatorTopSpeedScale {0.75};

    //--------  elevator constants  -------- 
    //fraction of maxMotorRotations for each major height: bottom, Y, X, A, top
    constexpr std::array<double, 5> elevatorMajorIncrements {0.0, 0.25, 0.5, 0.75, 1.0};
    constexpr int elevatorStepsPerSection {3}; 

    //intake position: motor rotations above the stowed position where the elevator sits to intake.
    //these are plain motor rotations, not measured heights (the DR4B's height per rotation is not constant)
    constexpr double elevatorIntakeRotations {1.3};
    //how many rotations the elevator lifts when the claw detects a piece
    constexpr double elevatorPieceLiftRotations {1.17};
    //gravity feedforward in millivolts: the voltage that just holds the lift still, at the bottom and at the top, blended
    //by position in between. measure it on the robot (hold it at each end and raise until it stops sagging). 0 = off,
    //the integral term then has to carry the load. a DR4B is normally heaviest to hold low, so expect Low > High. TODO tune
    constexpr double elevatorGravityLowMv {1575.0};
    constexpr double elevatorGravityHighMv {1500.0};
    //the motor positions those two voltages were measured at, and where the load starts at the bottom: the lowest
    //rotations are slack (unwinding the home tension), where turning the motor lifts nothing, so there is no gravity
    //load to hold. below elevatorLoadedStartRotations the feedforward is 0, ramping up to the Low value by LowAt.
    //the auto-tune (hold LEFT + RIGHT) measures all three. defaults match its test heights
    constexpr double elevatorGravityLowAtRotations {1.92};
    constexpr double elevatorGravityHighAtRotations {3.84};
    constexpr double elevatorLoadedStartRotations {0.88};

    //voltage to drive down with for both the startup home and the hold-B home, in volts (-12 to 12)
    constexpr double elevatorHomingVolts {-127.0/2};
    constexpr int elevatorHomingTimeMs {250};
    constexpr double elevatorZeroTolerance {0.5};
    constexpr int elevatorReturnTimeoutMs {4000};
    constexpr int elevatorHoldToHomeMs {300};
    //the startup home and hold-B home stop and zero on their own, checked only after elevatorHomeGraceMs of pushing.
    //if it never reaches the zero tension it gives up after elevatorManualHomeTimeoutMs without zeroing. TODO tune
    constexpr uint32_t elevatorHomeGraceMs {300};
    //the chain keeps tightening, so there is no hard stop: the home zeroes at a fixed tension instead.
    //push down at elevatorHomeMv (millivolts, low = gentle on the chain) until the highest motor current stays above
    //elevatorHomeStallCurrentMa for the confirm time. TODO tune the current from the "home ... mA" lines printed to the terminal
    constexpr int elevatorHomeMv {-3500};
    //measured at 3.5 V: free spin ~73 rpm at <60 mA, current plateaus ~1430 mA with velocity 0.0 on the stop
    constexpr int32_t elevatorHomeStallCurrentMa {1152};
    constexpr double elevatorHomeStallRpm {34.3};
    constexpr uint32_t elevatorHomeStallConfirmMs {150};
    constexpr uint32_t elevatorManualHomeTimeoutMs {8000};

    //elevator positioning
    //power while holding L1/L2 in manual, -127 to 127 for move()
    constexpr int elevatorManualPower {127};
    //manual down (L2) slows as it nears zero so it stops at the stow position instead of coasting below it:
    //within this many rotations of zero the power scales down to the min fraction. TODO tune
    constexpr double elevatorManualBottomTaperRotations {0.8};
    constexpr double elevatorManualBottomMinScale {0.2};

    //elevator voltage control: one voltage is computed and sent to BOTH motors so they can't fight each other.
    //TODO tune all of these on the robot
    //max voltage the elevator is ever given, in millivolts (12000 = full)
    constexpr double elevatorMaxVoltageMv {12000.0};
    //P gain: millivolts per rotation of error. 12000 = full power when 1+ rotation away, fades out as it gets close.
    //this sets how fast short moves are: one Up/Down step is only ~0.53 rotations, so a low gain means a low voltage
    constexpr double elevatorKp {14682.0};
    //smallest voltage that still moves the elevator (beats friction) so it doesn't stall just short of the target
    constexpr double elevatorMinVoltageMv {1710.0};
    //voltage slew limits, in millivolts per second. rise = speeding up (48000 = 0 to 12 V in 0.25 s),
    //fall = slowing down (72000 = 12 V to 0 in ~0.17 s). fall must stay fast enough to keep up with the P gain
    //dropping the voltage near the target, or the elevator overshoots. lower = smoother but slower to respond
    constexpr double elevatorVoltageRiseMvPerSec {48000.0};
    constexpr double elevatorVoltageFallMvPerSec {72000.0};
    //integral: only builds inside elevatorIntegralZone (rotations) so it can't wind up during big moves, and its voltage
    //is capped at elevatorIntegralMaxMv. it replaces the min voltage inside the zone: it grows until friction/gravity
    //are beaten, so the elevator lands on the target instead of stopping short. TODO tune
    constexpr double elevatorKi {237975.0};  //mV per (rotation * second)
    constexpr double elevatorIntegralZone {0.3};
    constexpr double elevatorIntegralMaxMv {3244.0};
    //derivative on measurement: mV per (rotation / second) of speed, brakes the approach so it doesn't overshoot. TODO tune
    constexpr double elevatorKd {587.0};
    //settled = within the tolerance AND slower than elevatorSettleSpeed (rot/s) for elevatorSettleDwellMs, then brake and hold.
    //requiring it to be slow is what makes the stop height repeatable instead of wherever it first touched the window
    constexpr double elevatorSettleTolerance {0.02};
    constexpr double elevatorSettleSpeed {0.05};
    constexpr uint32_t elevatorSettleDwellMs {100};
    //after settling, only start correcting again if it drifts this far. the gap between the two stops chatter
    constexpr double elevatorResettleTolerance {0.05};

    //--------  endEffector constants  -------- 
    //bench test only: right stick Y (+-127) times this gives pivot mV (47 * 127 = ~6000 mV), claw mV
    constexpr int endEffectorTestPivotMvPerStick {47};
    //claw homing: close at this low voltage until the speed drops under the stall rpm (after the grace time), zero there. TODO tune
    //3000 mV did not move the open claw at all. a stall only counts once the speed has gone over the moving rpm
    constexpr int endEffectorClawHomeMv {1500};
    constexpr double endEffectorClawHomeMovingRpm {15.0};
    constexpr std::uint32_t endEffectorClawHomeGraceMs {250};
    constexpr double endEffectorClawHomeStallRpm {5.0};
    constexpr std::uint32_t endEffectorClawHomeTimeoutMs {2000};
    //pivot homing: starts at its highest point, so drive up (positive) at this low voltage until it stalls, zero there
    //(so 0 is the top and lowering goes negative). TODO tune
    constexpr int endEffectorPivotHomeMv {3000};
    constexpr double endEffectorPivotHomeMovingRpm {10.0};
    constexpr std::uint32_t endEffectorPivotHomeGraceMs {250};
    constexpr double endEffectorPivotHomeStallRpm {5.0};
    constexpr std::uint32_t endEffectorPivotHomeTimeoutMs {2000};
    //horizontal position (also the standard scoring position), the average of the two pivot motors (A reads ~-610, B ~-602). 0 is the top
    constexpr double endEffectorPivotHorizontalDeg {-606.0};
    //stow position of the pivot (average of the two motors)
    constexpr double endEffectorPivotStowDeg {-115.0};
    constexpr int endEffectorPivotRpm {100}; //max speed for pivot moves, 200 is full for green
    constexpr int endEffectorClawRpm {100}; //max speed for claw moves, 200 is full for green
    //claw encoder position when open (degrees, 1:1 with the motor), closed is 0
    constexpr double endEffectorClawOpenDeg {212.0};

    //--------  command constants  --------


    //misc
    constexpr int radio {8};
    constexpr int imu {0};

    //motor gear ratios 
    constexpr pros::v5::MotorGears drivetrainGearRatio {pros::v5::MotorGears::green};
    constexpr pros::v5::MotorGears elevatorGearRatio {pros::v5::MotorGears::green};
    constexpr pros::v5::MotorGears endEffectorGears {pros::v5::MotorGears::green}; //green, not red: red made every move go twice as far

    inline pros::v5::Controller master({pros::E_CONTROLLER_MASTER});
    
}