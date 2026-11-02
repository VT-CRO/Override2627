#pragma once
#include <array>
#include <cstdint>

namespace config::auton{
    //--------  auton constants  --------
    //how long one LemLib move may take before it gives up. TODO tune
    constexpr int pathTimeoutMs {4000};
    //distance (inches) from the tracking center to the middle of the closed claw, straight ahead. TODO measure
    constexpr float clawReach {12.65};
    //elevator height (motor rotations) that carries the claw just above a red Alliance Goal (3.25" tall) while it
    //holds 0, 1, 2, 3 Pins already Placed there. measured
    constexpr std::array<double, 4> goalScoreRotations {1.52, 2.57, 3.56, 4.64};
    //scoring: drive straight ahead at this power (0-127) until the drive motors stall against the goal (both sides under
    //the stall rpm for the confirm time, after the grace time to get moving) or the timeout, then back off by
    //alignBackoff inches before dropping the Pin. the backoff sets where the claw ends up over the goal. TODO tune
    //(green motors: 35 power runs about 55 rpm free)
    constexpr int pushSpeed {35};
    constexpr double pushStallRpm {10.0};
    constexpr std::uint32_t pushGraceMs {300};
    constexpr std::uint32_t pushStallConfirmMs {100};
    constexpr int pushTimeoutMs {3000};
    //heading hold during the push (power per degree off, and per degree of change per 10 ms): gentle, since the push
    //power is low. raise P if it still meets the goal crooked, lower it (or raise D) if it weaves. TODO tune
    constexpr double pushHeadingKp {1.3};
    constexpr double pushHeadingKd {10};
    //the hold's strength at the elevator's top, as a fraction of the above (full when it's down, linear in between): the
    //higher layers swayed the robot into oscillating. TODO tune
    constexpr double pushHeadingTopScale {0.3};
    constexpr double alignBackoff {3.25};
    //how far (motor rotations) the elevator lowers from the scoring height before the claw opens. TODO tune
    constexpr double scoreDropRotations {0.5};
    //the claw opens once the elevator is within this many rotations of the lowered height (not fully settled). TODO tune
    constexpr double scoreDropToleranceRotations {0.12};
    //how long the pivot gets to angle down for a Pin, or back up to horizontal, before the claw moves. TODO tune
    constexpr std::uint32_t pivotMs {400};
    //how far (inches) the robot has to get from a goal after dropping the preload before the claw closes to re-home
    //(it re-homes on the move). TODO tune
    constexpr double rehomeClearance {1.0};
    //picking a stack up off the ground: the last pickupSlowIn inches before the claw is over it run at
    //pickupSpeed (0-127). TODO tune
    constexpr double pickupSlowIn {6.0};
    constexpr int pickupSpeed {50};
    //how long the claw gets to open or close before the robot moves. TODO tune
    constexpr std::uint32_t clawMs {300};
    //how long the claw gets to open off a scored Pin before the elevator lifts away. starts at clawMs; lower it if
    //the claw is clearly open sooner. TODO tune
    constexpr std::uint32_t clawOpenMs {300};
    //how far (inches) the robot backs from a goal after scoring before the elevator lowers to the intake height. TODO tune
    constexpr double intakeClearance {1.0};
    //how far (motor rotations) the elevator lifts a Match Load above the intake height while still in the Loader: enough
    //that it doesn't drag on the ground, not so much that the claw catches in the Loader. TODO tune
    constexpr double loadLiftRotations {0.2};
    //how far (inches) the robot backs straight away from a goal after scoring before it turns, so the claw clears the
    //Pin. TODO tune
    constexpr double scoreClearIn {8.0};
}
