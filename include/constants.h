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

    constexpr double elevatorZeroTolerance {0.5};
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
    constexpr int endEffectorClawHomeMv {2500};
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
    constexpr double endEffectorClawOpenDeg {205};
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
    //then before the pivot homes, the claw opens this much past the Pin grip (and waits this long) so the Pin can shift
    //out of the way and the pivot reaches its normal top stop. TODO tune
    constexpr double endEffectorClawPreloadReleaseDeg {20.0};
    constexpr std::uint32_t endEffectorClawPreloadReleaseMs {250};
    //picking up a lone Pin: the pivot angles down past horizontal before the claw closes. measured off the ground with
    //the elevator at its intake height
    constexpr double endEffectorPivotPinDeg {-779.5};

    //--------  command constants  --------

    //--------  auton constants  --------
    //trapezoidal motion profiles for autonomous drives and turns: speed up at the accel, cruise at the max speed, slow down
    //at the accel. lower = gentler. TODO tune
    //slow for testing. the earlier values were 24 in/s, 36 in/s^2, 180 deg/s, 360 deg/s^2
    constexpr double autonDriveMaxSpeed {8.0};    //inches per second
    constexpr double autonDriveAccel {12.0};      //inches per second^2 (0 to max in 0.67 s)
    constexpr double autonTurnMaxSpeed {60.0};    //degrees per second
    constexpr double autonTurnAccel {120.0};      //degrees per second^2 (0 to max in 0.5 s)
    //feedforward: output (out of 127) per unit of profile speed, so the robot keeps up with the profile instead of the PID
    //chasing it. estimated from the wheels' free speed (200 rpm, 3.25" wheels = 34 in/s, 10" track = 390 deg/s), TODO tune
    //drive: the estimate drove 14 in/s when the profile asked for 8 (overshooting every move), so scaled to match that
    //drive's constant feedforward (out of 127) that keeps the robot moving against friction. the full driveMinOutput kept
    //pushing as the profile slowed to a stop, and the weak PD needed ~2" of lead to cancel it, so every drive overshot by
    //~1.2". lower = less overshoot, but too low and it won't start moving. TODO tune
    constexpr double autonDriveKs {4.0};
    constexpr double autonDriveKv {127.0 / 80.0};
    //turn's constant feedforward, lowered from driveMinOutput (10) for the same reason as the drive's: turns ran 72-78 deg/s
    //on a 60 deg/s profile and overshot 1-1.8 deg, then snapped back. TODO tune
    constexpr double autonTurnKs {4.0};
    constexpr double autonTurnKv {127.0 / 390.0};
    //driveStraight's heading hold: output (out of 127) per degree off the starting heading, and per degree of change per
    //10 ms for damping. much stiffer than the turning gains, which are too weak to stop a drive veering. TODO tune
    constexpr double autonHeadingKp {7.0};
    constexpr double autonHeadingKd {20.0};
    //path.jerryio paths (pure pursuit): how far ahead on the path the robot aims (bigger = smoother but cuts corners),
    //and how long one path may take. the speeds along a path come from the path file. TODO tune
    constexpr float autonPathLookahead {10.0f};
    constexpr int autonPathTimeoutMs {5000};
    //distance (inches) from the tracking center to the middle of the closed claw, straight ahead. TODO measure
    constexpr float autonClawReach {11.75f};
    //elevator height (motor rotations) that carries the claw just above a red Alliance Goal (3.25" tall) while it
    //holds 0, 1, 2, 3 Pins already Placed there. measured
    constexpr std::array<double, 4> autonGoalScoreRotations {1.52, 2.57, 3.56, 4.64};    //driveTo skips its turn when the robot already faces the point within this many degrees (3 deg over a 25" drive is
    //about 1.3" sideways). TODO tune
    constexpr double autonSkipTurnDeg {3.0};
    //waypoint moves and the goal approach (which the aligner squares anyway) are looser: they skip a turn under this many
    //degrees (the drive just holds its heading, so the robot isn't turned back after the aligner squares it) and end a
    //turn once within the tolerance. TODO tune
    constexpr double autonLooseSkipTurnDeg {10.0};
    constexpr double autonLooseTurnTolDeg {4.0};
    //how much faster (times the profile's speed and acceleration) the moves that don't decide a position run: waypoints
    //and the long first part of a drive. the last autonPickupSlowIn inches into a stack stay at the tuned speed. TODO tune
    constexpr double autonFastScale {1.5};
    constexpr double autonPickupSlowIn {6.0};
    //scoring with the aligner: keep driving into the goal at the approach's own cruise power for this long, then back off by
    //this many inches before dropping the Pin. the backoff sets where the claw ends up over the goal. TODO tune
    constexpr std::uint32_t autonAlignMs {1300};
    constexpr double autonAlignBackoff {3.0};
    //how far (motor rotations) the elevator lowers from the scoring height before the claw opens. TODO tune
    constexpr double autonScoreDropRotations {0.3};
    //the claw opens once the elevator is within this many rotations of the lowered height (not fully settled). TODO tune
    constexpr double autonScoreDropToleranceRotations {0.12};
    //how long the pivot gets to angle down for a Pin, or back up to horizontal, before the claw moves. TODO tune
    constexpr std::uint32_t autonPivotMs {400};
    //how far (inches) the robot has to get from a goal after dropping the preload before the claw closes to re-home
    //(it re-homes on the move). TODO tune
    constexpr double autonRehomeClearance {6.0};
    //how long the claw gets to open or close before the robot moves. TODO tune
    constexpr std::uint32_t autonClawMs {300};


    //misc
    constexpr int radio {8};
    constexpr int imu {16};

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