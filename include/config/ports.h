#pragma once
#include "api.h"

namespace config::ports{
    //motor ports negative for init reverse motion
    
    //drivetrain motors
    constexpr int rightMotorOne {5}; 
    constexpr int rightMotorTwo {-8};
    constexpr int rightMotorThree {10};
    constexpr int leftMotorOne {-20};
    constexpr int leftMotorTwo {16};
    constexpr int leftMotorThree {-18};

    //elevator motors
    constexpr int leftElevator {14};
    constexpr int rightElevator {-3};

    //end effector (two pivot motors + claw motor)
    constexpr int pivotAPort {11};
    constexpr int pivotBPort {-1}; //negate if it spins opposite pivot A
    constexpr int clawPort {12};
    constexpr int imu {2};

    //odometry pods, ADI (top port, bottom port). encoders need an adjacent pair: A-B, C-D, E-F or G-H
    constexpr char vertPodTop {'A'};
    constexpr char vertPodBottom {'B'};
    constexpr char horizPodTop {'C'};
    constexpr char horizPodBottom {'D'};

    //motor gear ratios 
    constexpr pros::v5::MotorGears drivetrainGearRatio {pros::v5::MotorGears::green};
    constexpr pros::v5::MotorGears elevatorGearRatio {pros::v5::MotorGears::green};
    constexpr pros::v5::MotorGears endEffectorGearRatio {pros::v5::MotorGears::green}; //green, not red: red made every move go twice as far
}
