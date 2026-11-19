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
    constexpr std::uint32_t pivotHomeGraceMs {150};    //was 250
    constexpr double pivotHomeStallRpm {5.0};
    constexpr std::uint32_t pivotHomeTimeoutMs {2000};
    //horizontal position (also the standard scoring position), the average of the two pivot motors. 0 is the top
    //measured as -580 (A ~-610, B ~-602) with the green cartridges. the red ones (36:1 vs 18:1) are half the ratio, so this
    //is -580 / 2 as an estimate. TODO re-measure on the red cartridges
    constexpr double pivotHorizontalDeg {-290};
    //where the pivot swings to when the preload is clamped at the start of autonomous. it was -270 (20 deg above
    //horizontal), now 15 deg lower than that, 5 deg above horizontal. TODO tune
    constexpr double pivotPreloadDeg {-285};
    //stow position of the pivot (average of the two motors)
    constexpr double pivotStowDeg {-115.0};
    //homeBlocking waits for the pivot to get this close to stow (or time out) so the elevator doesn't move first
    constexpr double pivotStowToleranceDeg {10.0};
    constexpr std::uint32_t pivotStowTimeoutMs {2000};
    constexpr int pivotRpm {50};    //was 33 (the swings were slow); 50 is about a third faster //max speed for pivot moves, 100 is full for red (pivots are red now, was 200 for green). was 100, cut to a third for the 3x multiplier on the pivot
    constexpr int clawRpm {175}; //max speed for claw opening, 200 is full for green. was 100, which left the drop only partly open in clawOpenMs
    constexpr int clawCloseRpm {175}; //max speed for claw closing
    //claw encoder position when open (degrees, 1:1 with the motor), closed is 0. 155 opened so wide the gear skipped and
    //124 was too narrow, the Pins stuck in the claw, so it is between the two
    constexpr double clawOpenDeg {154};    //140 + 10%. 155 skipped the gear before
    //every closeClaw() aims this many degrees past the closed position (on top of any extra it is given), so it squeezes
    //what it holds. keep it small: a huge target (it was 1000) keeps the motor pushing at full torque, the claw gear
    //slides and the encoder loses where the claw really is, so later opens and closes do nothing
    constexpr double clawCloseExtraDeg {10.0};
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
