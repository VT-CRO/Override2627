#pragma once
#include <array>
#include <cstdint>

namespace config::drivetrain{
    //--------  drivetrain constants  -------- 
    //driver control acceleration limits, same idea as the elevator: the voltage may only change so fast.
    //only forward/back (translation) is limited, turning (rotation) is not, then the two are mixed into left/right.
    //in millivolts per second (12000 mV = full). rise = speeding up, fall = slowing down (lowering the voltage
    //on a spinning motor brakes it). blended from Low (elevator down) to High (elevator at top). lower = gentler. TODO tune
    //translation (~17% gentler than the first tune)
    constexpr double voltageRiseLowMvPerSec {25000.0};   //elevator down: 0 to 12 V in 0.48 s
    constexpr double voltageFallLowMvPerSec {25000.0};
    constexpr double voltageRiseHighMvPerSec {7500.0};   //elevator up: 0 to 12 V in 1.6 s
    constexpr double voltageFallHighMvPerSec {7500.0};
    //turning: stick scale and slew (fall faster than rise so it stops quickly). TODO tune with a driver
    //smallest stick output (past the deadband) that still overcomes friction; the speed scaling never shrinks it
    constexpr double minOutput {10.0};
    constexpr double turnScale {0.7};
    constexpr double turnRiseMvPerSec {40000.0};
    constexpr double turnFallMvPerSec {60000.0};
    //top speed (fraction of full) with the elevator at the top, fading linearly from 1.0 with the elevator down. TODO tune
    constexpr double elevatorTopSpeedScale {0.75};
}
