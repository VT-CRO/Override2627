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
    constexpr int leftElevator {6};
    constexpr int rightElevator {-19};
    constexpr double elevatorMaxMotorRotations {6.4};

    //end effector motors
    constexpr int leftEndEffectorMotor {10};
    constexpr int rightEndEffectorMotor {}; //not plugged in need to find a port

    //--------  drivetrain constants  -------- 
    //driver control acceleration limits, same idea as the elevator: the voltage may only change so fast.
    //only forward/back (translation) is limited, turning (rotation) is not, then the two are mixed into left/right.
    //in millivolts per second (12000 mV = full). rise = speeding up, fall = slowing down (lowering the voltage
    //on a spinning motor brakes it). blended from Low (elevator down) to High (elevator at top). lower = gentler. TODO tune
    //translation (~17% gentler than the first tune)
    constexpr double driveVoltageRiseLowMvPerSec {25000.0};   //elevator down: 0 to 12 V in 0.48 s
    constexpr double driveVoltageFallLowMvPerSec {25000.0};
    constexpr double driveVoltageRiseHighMvPerSec {12500.0};  //elevator up: 0 to 12 V in 0.96 s
    constexpr double driveVoltageFallHighMvPerSec {12500.0};

    //--------  elevator constants  -------- 
    //fraction of maxMotorRotations for each major height: bottom, Y, X, A, top
    constexpr std::array<double, 5> elevatorMajorIncrements {0.0, 0.25, 0.5, 0.75, 1.0};
    constexpr int elevatorStepsPerSection {3}; 

    //voltage to drive down with for both the startup home and the hold-B home, in volts (-12 to 12)
    constexpr double elevatorHomingVolts {-127.0/2};
    constexpr int elevatorHomingTimeMs {250};
    constexpr double elevatorZeroTolerance {0.5};
    constexpr int elevatorReturnTimeoutMs {4000};
    constexpr int elevatorHoldToHomeMs {300};

    //elevator positioning
    //power while holding L1/L2 in manual, -127 to 127 for move()
    constexpr int elevatorManualPower {127};

    //elevator voltage control: one voltage is computed and sent to BOTH motors so they can't fight each other.
    //TODO tune all of these on the robot
    //max voltage the elevator is ever given, in millivolts (12000 = full)
    constexpr double elevatorMaxVoltageMv {12000.0};
    //P gain: millivolts per rotation of error. 12000 = full power when 1+ rotation away, fades out as it gets close.
    //this sets how fast short moves are: one Up/Down step is only ~0.53 rotations, so a low gain means a low voltage
    constexpr double elevatorKp {12000.0};
    //smallest voltage that still moves the elevator (beats friction) so it doesn't stall just short of the target
    constexpr double elevatorMinVoltageMv {800.0};
    //voltage slew limits, in millivolts per second. rise = speeding up (48000 = 0 to 12 V in 0.25 s),
    //fall = slowing down (72000 = 12 V to 0 in ~0.17 s). fall must stay fast enough to keep up with the P gain
    //dropping the voltage near the target, or the elevator overshoots. lower = smoother but slower to respond
    constexpr double elevatorVoltageRiseMvPerSec {48000.0};
    constexpr double elevatorVoltageFallMvPerSec {72000.0};
    //within this many rotations of the target: stop driving and brake (hold)
    constexpr double elevatorSettleTolerance {0.05};
    //after settling, only start correcting again if it drifts this far. the gap between the two stops chatter
    constexpr double elevatorResettleTolerance {0.15};

    //--------  endEffector constants  -------- 
    //max speed 200 rpm
    constexpr int endEffectorMoveVelocity {50}; 

    //these are in terms of degress since vex is butt and makes it hareder for radians :(
    constexpr int endEffectorBottomAngle {0};
    constexpr int endEffectorHorizonatalDeg {60};
    constexpr int endEffectorTopDeg {120};
    constexpr int endEffectorHardStopDeg {190};
    constexpr int endEffectorHomingTimeMs {1000};
    constexpr int endEffectorHomingPower {40};

    //--------  command constants  -------- 
    //score: how long after R2 the prongs wait before dropping, in ms. the elevator starts coming down right away
    constexpr std::uint32_t scoreDropDelayMs {750};


    //misc
    constexpr int radio {8};
    constexpr int imu {0};

    //motor gear ratios 
    constexpr pros::v5::MotorGears drivetrainGearRatio {pros::v5::MotorGears::green};
    constexpr pros::v5::MotorGears elevatorGearRatio {pros::v5::MotorGears::green};
    constexpr pros::v5::MotorGears endEffectorGears {pros::v5::MotorGears::green}; //green, not red: red made every move go twice as far

    inline pros::v5::Controller master({pros::E_CONTROLLER_MASTER});
    
}