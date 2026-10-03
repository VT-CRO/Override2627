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
    //max speed for move_absolute, in rpm (green cartridge tops out at 200)
    constexpr int elevatorMoveVelocity {200};
    //power while holding Up/Down in manual, -127 to 127 for move()
    constexpr int elevatorManualPower {127};

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
    constexpr std::uint32_t scoreDropDelayMs {500};


    //misc
    constexpr int radio {8};
    constexpr int imu {0};

    //motor gear ratios 
    constexpr pros::v5::MotorGears drivetrainGearRatio {pros::v5::MotorGears::green};
    constexpr pros::v5::MotorGears elevatorGearRatio {pros::v5::MotorGears::green};
    constexpr pros::v5::MotorGears endEffectorGears {pros::v5::MotorGears::green}; //green, not red: red made every move go twice as far

    inline pros::v5::Controller master({pros::E_CONTROLLER_MASTER});
    
}