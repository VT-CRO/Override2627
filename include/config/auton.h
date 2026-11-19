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
    //holds 0, 1, 2, 3, 4 Pins already Placed there. measured as {1.52, 2.57, 3.56, 4.64}, the fifth (5.69) continues the
    //roughly 1.05 rotations per layer and is not measured. then each raised 0.2, then 0.1 more, then 0.2 less. the first
    //(the preload) was then raised 0.1 more, 1.62 to 1.72. then the first three (layers 0, 1, 2) were raised 0.15 more.
    //then the first three were raised 0.4 more, then lowered 0.6. TODO measure
    constexpr std::array<double, 5> goalScoreRotations {1.52, 2.77, 3.76, 4.84, 5.99};    //layers 1, 2, 3 (pins 2, 3, 4) raised 0.2, then layer 4 (pin 5) raised 0.2
    //scoring: drive straight ahead at this power (0-127) until the drive motors stall against the goal (both sides under
    //the stall rpm for the confirm time, after the grace time to get moving) or the timeout, then back off by
    //alignBackoff inches before dropping the Pin. the backoff sets where the claw ends up over the goal. TODO tune
    //(green motors: 35 power runs about 55 rpm free)
    constexpr int pushSpeed {46};    //was 35, then 15% faster (40), then 15% faster again (46)
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
    //score() backs off this much (inches) further than alignBackoff before it drops the Pin. TODO tune
    constexpr double scoreExtraBackoff {0.5};    //was 0.35, +0.15
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
    //was 6 in at 50, which knocked the Pin over on the skills stack, so it slows earlier and to less
    constexpr double pickupSlowIn {10.0};
    constexpr int pickupSpeed {32};    //was 35, then 15% faster (40), then 20% slower (32)
    //how long the claw gets to open or close before the robot moves. TODO tune
    constexpr std::uint32_t clawMs {300};
    //how long the claw gets to open off a scored Pin before the elevator lifts away. starts at clawMs; lower it if
    //the claw is clearly open sooner. TODO tune
    constexpr std::uint32_t clawOpenMs {175};    //was 300, the claw opens at 175 rpm now
    //how far (inches) the robot backs from a goal after scoring before the elevator lowers to the intake height. TODO tune
    constexpr double intakeClearance {1.0};
    //how far (motor rotations) the elevator lifts a Match Load above the intake height while still in the Loader: enough
    //that it doesn't drag on the ground, not so much that the claw catches in the Loader. TODO tune
    constexpr double loadLiftRotations {0.2};
    //how far (inches) the robot backs straight away from a goal after scoring before it turns, so the claw clears the
    //Pin. TODO tune
    constexpr double scoreClearIn {8.0};
}
