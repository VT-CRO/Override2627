#pragma once
#include <array>
#include <cstdint>

namespace config::endeffector{
    //--------  endEffector constants  -------- 
    //bench test only: right stick Y (+-127) times this gives pivot mV (47 * 127 = ~6000 mV), claw mV
    constexpr int testPivotMvPerStick {47};
    //claw homing: close at this low voltage until the speed drops under the stall rpm (after the grace time, which covers
    //spinning up), zero there. the timeout zeroes it anyway, so it must cover a full close from open. TODO tune
    //3000 mV did not move the open claw at all: the voltage has to break it free, or it zeroes wherever it stuck
    //1500 stalled the claw about 40 deg short of its stop on the re-home after the drop (it slowed to a crawl partway), so raised
    constexpr int clawHomeMv {4000};
    constexpr std::uint32_t clawHomeGraceMs {250};
    constexpr double clawHomeStallRpm {5.0};
    constexpr std::uint32_t clawHomeTimeoutMs {2000};
    //pivot homing: starts at its highest point, so drive up (positive) at this low voltage until it stalls, zero there
    //(so 0 is the top and lowering goes negative). the timeout zeroes it anyway. TODO tune
    constexpr int pivotHomeMv {3000};
    constexpr std::uint32_t pivotHomeGraceMs {250};
    constexpr double pivotHomeStallRpm {5.0};
    constexpr std::uint32_t pivotHomeTimeoutMs {2000};
    //horizontal position (also the standard scoring position), the average of the two pivot motors (A reads ~-610, B ~-602). 0 is the top
    constexpr double pivotHorizontalDeg {-580};
    //stow position of the pivot (average of the two motors)
    constexpr double pivotStowDeg {-115.0};
    //homeBlocking waits for the pivot to get this close to stow (or time out) so the elevator doesn't move first
    constexpr double pivotStowToleranceDeg {10.0};
    constexpr std::uint32_t pivotStowTimeoutMs {2000};
    constexpr int pivotRpm {100}; //max speed for pivot moves, 200 is full for green
    constexpr int clawRpm {100}; //max speed for claw moves, 200 is full for green
    //claw encoder position when open (degrees, 1:1 with the motor), closed is 0
    constexpr double clawOpenDeg {155};
    //homing with a preload Pin in the claw: the claw stalls on the Pin instead of its stop. this is where it stalls, in
    //the normal (empty home) degrees, so its zero can be shifted to match. measure: home empty, then close the claw on a
    //Pin and read the claw. measured
    constexpr double clawPinGripDeg {14.0};
    //voltage the claw closes at while homing on the preload Pin (more than the empty home's, for a firmer grip). the Pin
    //compresses more at a higher voltage, so re-measure the grip above if this changes a lot. TODO tune
    constexpr int clawClampMv {3000};
    //the wiggle on the way to scoring the preload: open this many degrees from the Pin grip, then shut again, this many
    //times, each half taking this long. settles a Pin stuck at an awkward angle. TODO tune
    constexpr double clawWiggleDeg {4.0};
    constexpr int clawWiggles {2};
    constexpr std::uint32_t clawWiggleMs {150};
}
