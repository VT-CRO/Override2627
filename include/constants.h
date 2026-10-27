#pragma once
#include "api.h"
#include "lemlib/api.hpp"
#include <array>
#include <cstdint>

namespace constants{
    //motor ports negative for init reverse motion
    
    //drivetrain motors
    constexpr int rightMotorOne {6}; 
    constexpr int rightMotorTwo {-10};
    constexpr int rightMotorThree {8};
    constexpr int leftMotorOne {-18};
    constexpr int leftMotorTwo {19};
    constexpr int leftMotorThree {-20};

    //elevator motors
    constexpr int leftElevator {11};
    constexpr int rightElevator {-1};
    constexpr double elevatorMaxMotorRotations {6.4};

    //end effector (two pivot motors + claw motor)
    constexpr int endEffectorPivotAPort {12};
    constexpr int endEffectorPivotBPort {-2}; //negate if it spins opposite pivot A
    constexpr int endEffectorClawPort {13};

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
    constexpr int elevatorReturnTimeoutMs {4000};
    //the home (X, or autonomous) stops and zeroes on its own, checked only after elevatorHomeGraceMs of pushing.
    //if it never reaches the zero tension it zeroes anyway after elevatorManualHomeTimeoutMs. TODO tune
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
    //claw homing: close at this low voltage until the speed drops under the stall rpm (after the grace time, which covers
    //spinning up), zero there. the timeout zeroes it anyway, so it must cover a full close from open. TODO tune
    //3000 mV did not move the open claw at all: the voltage has to break it free, or it zeroes wherever it stuck
    //1500 stalled the claw about 40 deg short of its stop on the re-home after the drop (it slowed to a crawl partway), so raised
    constexpr int endEffectorClawHomeMv {4000};
    constexpr std::uint32_t endEffectorClawHomeGraceMs {250};
    constexpr double endEffectorClawHomeStallRpm {5.0};
    constexpr std::uint32_t endEffectorClawHomeTimeoutMs {2000};
    //pivot homing: starts at its highest point, so drive up (positive) at this low voltage until it stalls, zero there
    //(so 0 is the top and lowering goes negative). the timeout zeroes it anyway. TODO tune
    constexpr int endEffectorPivotHomeMv {3000};
    constexpr std::uint32_t endEffectorPivotHomeGraceMs {250};
    constexpr double endEffectorPivotHomeStallRpm {5.0};
    constexpr std::uint32_t endEffectorPivotHomeTimeoutMs {2000};
    //horizontal position (also the standard scoring position), the average of the two pivot motors (A reads ~-610, B ~-602). 0 is the top
    constexpr double endEffectorPivotHorizontalDeg {-580};
    //stow position of the pivot (average of the two motors)
    constexpr double endEffectorPivotStowDeg {-115.0};
    //homeBlocking waits for the pivot to get this close to stow (or time out) so the elevator doesn't move first
    constexpr double endEffectorPivotStowToleranceDeg {10.0};
    constexpr std::uint32_t endEffectorPivotStowTimeoutMs {2000};
    constexpr int endEffectorPivotRpm {100}; //max speed for pivot moves, 200 is full for green
    constexpr int endEffectorClawRpm {100}; //max speed for claw moves, 200 is full for green
    //claw encoder position when open (degrees, 1:1 with the motor), closed is 0
    constexpr double endEffectorClawOpenDeg {155};
    //homing with a preload Pin in the claw: the claw stalls on the Pin instead of its stop. this is where it stalls, in
    //the normal (empty home) degrees, so its zero can be shifted to match. measure: home empty, then close the claw on a
    //Pin and read the claw. measured
    constexpr double endEffectorClawPinGripDeg {14.0};
    //voltage the claw closes at while homing on the preload Pin (more than the empty home's, for a firmer grip). the Pin
    //compresses more at a higher voltage, so re-measure the grip above if this changes a lot. TODO tune
    constexpr int endEffectorClawClampMv {3000};
    //the wiggle on the way to scoring the preload: open this many degrees from the Pin grip, then shut again, this many
    //times, each half taking this long. settles a Pin stuck at an awkward angle. TODO tune
    constexpr double endEffectorClawWiggleDeg {4.0};
    constexpr int endEffectorClawWiggles {2};
    constexpr std::uint32_t endEffectorClawWiggleMs {150};

    //--------  command constants  --------

    //--------  auton constants  --------
    //how long one LemLib move may take before it gives up. TODO tune
    constexpr int autonPathTimeoutMs {4000};
    //distance (inches) from the tracking center to the middle of the closed claw, straight ahead. TODO measure
    constexpr float autonClawReach {12.65};
    //elevator height (motor rotations) that carries the claw just above a red Alliance Goal (3.25" tall) while it
    //holds 0, 1, 2, 3 Pins already Placed there. measured
    constexpr std::array<double, 4> autonGoalScoreRotations {1.52, 2.57, 3.56, 4.64};
    //scoring: drive straight ahead at this power (0-127) until the drive motors stall against the goal (both sides under
    //the stall rpm for the confirm time, after the grace time to get moving) or the timeout, then back off by
    //autonAlignBackoff inches before dropping the Pin. the backoff sets where the claw ends up over the goal. TODO tune
    //(green motors: 35 power runs about 55 rpm free)
    constexpr int autonPushSpeed {35};
    constexpr double autonPushStallRpm {10.0};
    constexpr std::uint32_t autonPushGraceMs {300};
    constexpr std::uint32_t autonPushStallConfirmMs {100};
    constexpr int autonPushTimeoutMs {3000};
    //heading hold during the push (power per degree off, and per degree of change per 10 ms): gentle, since the push
    //power is low. raise P if it still meets the goal crooked, lower it (or raise D) if it weaves. TODO tune
    constexpr double autonPushHeadingKp {1.3};
    constexpr double autonPushHeadingKd {10};
    //the hold's strength at the elevator's top, as a fraction of the above (full when it's down, linear in between): the
    //higher layers swayed the robot into oscillating. TODO tune
    constexpr double autonPushHeadingTopScale {0.3};
    constexpr double autonAlignBackoff {3.25};
    //how far (motor rotations) the elevator lowers from the scoring height before the claw opens. TODO tune
    constexpr double autonScoreDropRotations {0.5};
    //the claw opens once the elevator is within this many rotations of the lowered height (not fully settled). TODO tune
    constexpr double autonScoreDropToleranceRotations {0.12};
    //how long the pivot gets to angle down for a Pin, or back up to horizontal, before the claw moves. TODO tune
    constexpr std::uint32_t autonPivotMs {400};
    //how far (inches) the robot has to get from a goal after dropping the preload before the claw closes to re-home
    //(it re-homes on the move). TODO tune
    constexpr double autonRehomeClearance {1.0};
    //picking a stack up off the ground: the last autonPickupSlowIn inches before the claw is over it run at
    //autonPickupSpeed (0-127). TODO tune
    constexpr double autonPickupSlowIn {6.0};
    constexpr int autonPickupSpeed {50};
    //how long the claw gets to open or close before the robot moves. TODO tune
    constexpr std::uint32_t autonClawMs {300};
    //how long the claw gets to open off a scored Pin before the elevator lifts away. starts at autonClawMs; lower it if
    //the claw is clearly open sooner. TODO tune
    constexpr std::uint32_t autonClawOpenMs {300};
    //how far (inches) the robot backs from a goal after scoring before the elevator lowers to the intake height. TODO tune
    constexpr double autonIntakeClearance {1.0};
    //how far (motor rotations) the elevator lifts a Match Load above the intake height while still in the Loader: enough
    //that it doesn't drag on the ground, not so much that the claw catches in the Loader. TODO tune
    constexpr double autonLoadLiftRotations {0.2};
    //how far (inches) the robot backs straight away from a goal after scoring before it turns, so the claw clears the
    //Pin. TODO tune
    constexpr double autonScoreClearIn {8.0};


    //misc
    constexpr int radio {5};
    constexpr int imu {15};

    //odometry pods, ADI (top port, bottom port). encoders need an adjacent pair: A-B, C-D, E-F or G-H
    constexpr char vertPodTop {'A'};
    constexpr char vertPodBottom {'B'};
    constexpr char horizPodTop {'C'};
    constexpr char horizPodBottom {'D'};

    //motor gear ratios 
    constexpr pros::v5::MotorGears drivetrainGearRatio {pros::v5::MotorGears::green};
    constexpr pros::v5::MotorGears elevatorGearRatio {pros::v5::MotorGears::green};
    constexpr pros::v5::MotorGears endEffectorGears {pros::v5::MotorGears::green}; //green, not red: red made every move go twice as far

    inline pros::v5::Controller master({pros::E_CONTROLLER_MASTER});
    
}