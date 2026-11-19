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
    //tracking center to the goal's center once stalled against it on a straight approach (0, 90, 180 deg), where the
    //robot reaches closer than on a diagonal one (clawReach - alignBackoff = 9.4, tuned at 45 deg). 8.4 is an estimate
    //(about 1 in closer, from the robot backing out 1 in short at -90). TODO measure/tune
    constexpr double contactStraightIn {8.4};
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
    //Red A goal to the loader row, along the field's y axis: goal y 46.66 minus loader y 11.44 (docs/field-coordinates.md).
    //in the goal frame score() sets, that is the local x the robot must reach to be level with the loader center. every
    //trip to the loader uses it, so tune this one number if the claw is off the loader center. TODO tune
    //on the 5th score of each goal (layer 4) the wrist rides this many degrees above horizontal while the elevator climbs,
    //because the elevator was not at height when the robot reached the goal; it comes back down to horizontal once the
    //elevator is within wristDownNearRotations of the height. up is positive. TODO tune
    constexpr double wristUpDeg {70};
    constexpr double wristDownNearRotations {0.5};
    //how far (inches) every loader trip is shifted along the wall toward field +y, off the loader's exact center: dead center
    //lets the claw jam on the loader's two side braces, with the claw a bit off center one jaw goes outside a brace and the
    //other stays inside, which picks the Pin up consistently. 1 cm. TODO tune
    constexpr double loaderShiftIn {0.394};
    constexpr double loaderLineFromGoalIn {35.22 - loaderShiftIn};    //local +x is field south, so +y field is minus here
    //how far (inches) the robot drives into the loader. it stalls on the loader, this just has to exceed the real travel
    constexpr double loaderDriveIn {11.5 - 0.394};    //was 11.5, 1 cm less so the claw goes in a bit less
    //Red B's center in the frame score() sets on Red A (goal at (0, 0), +x field south, +y field east): Red B (46.66, 23.11)
    //minus Red A (23.11, 46.66) = 23.55 east, 23.55 south (docs/field-coordinates.md)
    constexpr double redBFromRedAX {23.55};
    constexpr double redBFromRedAY {23.55};
    //the loader trip from Red B, in the frame score() sets on Red B (goal at (0, 0), +x field south, +y field east).
    //row: Red B y 23.11 minus loader y 11.44 = 11.67 south of the goal. docs/field-coordinates.md
    constexpr double redBLoaderRowX {11.67 - loaderShiftIn};
    //Red B frame back-out point: the same 23.11 in from the wall as the Red A trips (46.66 - 23.11 = 23.55 west of the goal)
    constexpr double redBLoaderBackY {-23.55};
}
