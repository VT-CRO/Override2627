#include "auton/auton.h"
#include "config/auton.h"
#include "config/drivetrain.h"
#include "config/elevator.h"
#include "config/endeffector.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace cfg = config::auton;

Auton::Auton(pros::v5::Controller& master, Drivetrain& drivetrain, Elevator& elevator, EndEffector& endEffector)
    : master(master), drivetrain(drivetrain), chassis(drivetrain.chassis), elevator(elevator), endEffector(endEffector) {}

namespace {
//coordinates in inches relative to where the robot starts: its starting spot is (0, 0) facing heading 0, +y is
//forward from the start, +x is to its right. LemLib headings: 0 along +y, clockwise positive. the spots were worked
//out from the field figure (start at (8.5, 64.955) facing the blue Alliance Station, goals at (23.11, 46.66) and
//(46.66, 23.11) from the red corner), so expect them to be off by an inch or two on a real field

//LemLib's maxSpeed is 0-127 (TODO tune on the robot)
const lemlib::MoveToPointParams back {.forwards = false};
//backing out to a spot that is only there to reposition: hand over to the next turn within 2 in, still moving,
//instead of stopping (the turn and pickup after it aim from wherever the robot ends up). TODO tune
const lemlib::MoveToPointParams backThrough {.forwards = false, .minSpeed = 30, .earlyExitRange = 2};
//every turn is followed by a move that steers or aligns itself (moveToPoint, or the push into the goal's aligner), so
//turns hand over once within 3 deg instead of settling to 1 deg (minSpeed 1 only turns the early exit on)
const lemlib::TurnToHeadingParams turn {.minSpeed = 1, .earlyExitRange = 3};

constexpr int timeoutMs {cfg::pathTimeoutMs};

//tracking center to the goal's center once stalled against it: score() backs off alignBackoff so the claw is
//over the goal, so the robot must have stopped that far past the claw-over spot
//(this is the value at 45 deg, where it was tuned)
constexpr double contactDiagonalIn {cfg::clawReach - cfg::alignBackoff};

//the same distance at the stalled heading: the robot reaches closer on a straight approach (0, 90, 180) than on a
//diagonal one, so it goes linearly from contactStraightIn at a straight heading to contactDiagonalIn at 45 deg off
double contactAt(double headingDeg){
    double offStraight {std::fabs(std::remainder(headingDeg, 90.0))};    //0 to 45
    return cfg::contactStraightIn + (contactDiagonalIn - cfg::contactStraightIn) * offStraight / 45.0;
}
}

//LemLib turnToPoint. returns the heading from where the turn ended to (x, y), for score() to hold through the push
//(not the heading at the start: after a backThrough the robot is still rolling as it turns). maxSpeed (0-127) slows
//just this turn, e.g. to give the elevator time to rise before the push
double Auton::facePoint(double x, double y, int maxSpeed){
    chassis.turnToPoint(x, y, timeoutMs, {.maxSpeed = maxSpeed, .minSpeed = 1, .earlyExitRange = 3}, false);
    lemlib::Pose end {chassis.getPose()};
    return lemlib::radToDeg(std::atan2(x - end.x, y - end.y));
}

//wait until the elevator is within toleranceRotations of `rotations`, without waiting for it to fully settle
void Auton::waitElevatorNear(double rotations, double toleranceRotations, std::uint32_t waitMs){
    std::uint32_t start {pros::millis()};
    while(std::abs(elevator.rotations() - rotations) > toleranceRotations && pros::millis() - start < waitMs){
        pros::delay(10);
    }
}

//set the Pin down on the goal (which already has `layer` Pins on it) and let go. no lift back up afterwards: the
//open claw slides off the Pin as the robot backs straight away, and intakeWhenClear() lowers it from here
void Auton::release(int layer, double openExtraDeg){
    //it has been rising since the pickup, all through the push
    std::uint32_t arriveStart {pros::millis()};
    waitElevatorNear(cfg::goalScoreRotations[layer], cfg::scoreDropToleranceRotations);
    std::uint32_t arrivedMs {pros::millis() - arriveStart};
    double dropTo {cfg::goalScoreRotations[layer] - cfg::scoreDropRotations};
    elevator.requestHeight(dropTo);
    std::uint32_t lowerStart {pros::millis()};
    waitElevatorNear(dropTo, cfg::scoreDropToleranceRotations);
    std::printf("release layer %d: waited %u ms for the elevator to arrive, %u ms to lower to the drop height\n", layer,
                static_cast<unsigned>(arrivedMs), static_cast<unsigned>(pros::millis() - lowerStart));    endEffector.waitForClawHome();    //the preload home finishes on the way here
    endEffector.openClaw(openExtraDeg);    //wider than normal so the claw clears the Pin. TODO tune
    pros::delay(cfg::clawOpenMs);
}

//the robot is already at the goal (the caller drove it in, nothing is pushed here): assume it is touching the goal along
//`heading`, reset the pose so the goal's center is (0, 0), back off to where the claw is over the goal, and drop the
//Pin. everything after it is relative to the goal. pushSpeed is no longer used
void Auton::score(int layer, double heading, double extraBackoffIn, int pushSpeed, double openExtraDeg){
    (void)heading;
    (void)pushSpeed;
    lemlib::Pose stalled {chassis.getPose()};
    double rad {lemlib::degToRad(stalled.theta)};
    //where odometry thinks the goal is. after the first score this should be (0, 0): the gap is how far odometry
    //drifted (or the robot stalled somewhere else) since the last reset
    double contactIn {contactAt(stalled.theta)};
    double foundX {stalled.x + contactIn * std::sin(rad)};
    double foundY {stalled.y + contactIn * std::cos(rad)};
    std::printf("score %d: goal found %.2f in from where odometry had it (heading %.1f, contact %.2f in)\n", layer,
                std::hypot(foundX, foundY), stalled.theta, contactIn);
    scoreShiftX = -contactIn * std::sin(rad) - stalled.x;    //remember how far the origin moved, for going back to old points
    scoreShiftY = -contactIn * std::cos(rad) - stalled.y;
    chassis.setPose(-contactIn * std::sin(rad), -contactIn * std::cos(rad), stalled.theta);
    //extraBackoffIn: backs off this much more for one call. the contact difference keeps the drop the same distance from
    //where the robot stalled as when contact was always the diagonal value (the drop was tuned that way)
    double backTo {cfg::clawReach + cfg::scoreExtraBackoff + extraBackoffIn + (contactIn - contactDiagonalIn)};
    std::uint32_t backStart {pros::millis()};
    //the back-up is short, so it exits within 1.5 in of the point at minSpeed instead of slowing to a stop (that tail was
    //most of its 550 ms), then the motors are stopped. the stop carries about 1 in, so it lands on the point. was 40 and
    //0.5 in: it stopped 0.5 in too far back and twisted off the straight line braking from that speed
    chassis.moveToPoint(-backTo * std::sin(rad), -backTo * std::cos(rad), timeoutMs,
                        {.forwards = false, .maxSpeed = 114, .minSpeed = 30, .earlyExitRange = 1.5}, false);
    chassis.tank(0, 0, true);
    std::uint32_t releaseStart {pros::millis()};
    release(layer, openExtraDeg);
    std::printf("score %d: back-up drive took %u ms, release took %u ms\n", layer, static_cast<unsigned>(releaseStart - backStart),
                static_cast<unsigned>(pros::millis() - releaseStart));}

//LemLib moveToPoint straight back along the current heading, so the claw doesn't swing into the Pin it just dropped.
//(the spot depends on where the push stalled, so it can't be a fixed coordinate)
//timeoutOverrideMs: give up after this long instead of pathTimeoutMs (0 = use pathTimeoutMs)
void Auton::backStraight(double inches, const lemlib::MoveToPointParams& params, int timeoutOverrideMs){
    lemlib::Pose pose {chassis.getPose()};
    double rad {lemlib::degToRad(pose.theta)};
    chassis.moveToPoint(pose.x - inches * std::sin(rad), pose.y - inches * std::cos(rad),
                        timeoutOverrideMs > 0 ? timeoutOverrideMs : timeoutMs, params, false);
}

//true once the robot has moved `inches` from where it is now
std::function<bool()> Auton::movedFromHere(double inches){
    lemlib::Pose from {chassis.getPose()};
    return [this, from, inches]{
        lemlib::Pose pose {chassis.getPose()};
        return std::hypot(pose.x - from.x, pose.y - from.y) >= inches;
    };
}

//lower the elevator to the intake height once the robot has backed intakeClearance from the goal (any sooner
//and the claw could catch the Pin), in the background so it is down by the time the robot reaches the next stack
void Auton::intakeWhenClear(){
    pros::Task([this, clear = movedFromHere(cfg::intakeClearance)]{
        std::uint32_t start {pros::millis()};
        while(!clear() && pros::millis() - start < 3000){
            pros::delay(10);
        }
        elevator.requestIntake();
    });
}

//raise the elevator to `rotations` once the robot has backed `inches` from where it is now (so the Pin is clear of the
//matchloader first), in the background. the lift then runs while the robot finishes backing up and turns to the goal
//wristUp: also raises the wrist wristUpDeg above horizontal while the elevator climbs, and lowers it back to horizontal once
//the elevator is within wristDownNearRotations of the height (the top layers are slow to arrive)
void Auton::liftWhenClear(double rotations, double inches, bool wristUp){
    pros::Task([this, rotations, wristUp, clear = movedFromHere(inches)]{
        std::uint32_t start {pros::millis()};
        while(!clear() && pros::millis() - start < 3000){
            pros::delay(10);
        }
        elevator.requestHeight(rotations);
        if(wristUp){
            endEffector.pivotTo(config::endeffector::pivotHorizontalDeg + cfg::wristUpDeg);
            start = pros::millis();
            while(std::abs(elevator.rotations() - rotations) > cfg::wristDownNearRotations && pros::millis() - start < 4000){
                pros::delay(10);
            }
            endEffector.pivotTo(config::endeffector::pivotHorizontalDeg);
        }
    });
}

//turn to face (x, y), drive in until the claw is over it, close the claw, and lift to liftTo (motor rotations).
//clawFirst: let the claw close before the elevator moves (in the Loader, where lifting early can jam the claw);
//otherwise the lift starts as the claw closes, so it rises while the robot turns and drives to the goal
void Auton::pickUp(double x, double y, double liftTo, bool clawFirst, double closeExtraDeg){
    facePoint(x, y);
    //the claw has to be down at the intake height before it drives in. coming down from a high score can take longer
    //than the back-up and turn, so stop and wait if it isn't there yet (near is enough). the stop matters: the turn
    //exits early and leaves the motors running
    if(std::abs(elevator.rotations() - config::elevator::intakeRotations) > config::elevator::resettleTolerance){
        chassis.tank(0, 0, true);
        waitElevatorNear(config::elevator::intakeRotations, config::elevator::resettleTolerance, 2000);
    }

    //stop clawReach short of (x, y): full speed until pickupSlowIn before that, then the rest at pickupSpeed
    lemlib::Pose pose {chassis.getPose()};
    double distance {std::hypot(x - pose.x, y - pose.y)};
    double ux {(x - pose.x) / distance};
    double uy {(y - pose.y) / distance};
    double stopAt {distance - cfg::clawReach};
    double slowAt {stopAt - cfg::pickupSlowIn};
    if(slowAt > 0){
        //hand over to the slow part within 1 in, without stopping
        chassis.moveToPoint(pose.x + slowAt * ux, pose.y + slowAt * uy, timeoutMs,
                            {.minSpeed = cfg::pickupSpeed, .earlyExitRange = 1}, false);
    }
    //the slow part is open loop at pickupSpeed, holding the heading toward the stack, until the claw is over it. not a
    //moveToPoint with a maxSpeed: LemLib raises that cap to at least 60 once within 7.5 in of the target, so the robot
    //crept in and then lunged at the stack and knocked it over
    double heading {lemlib::radToDeg(std::atan2(ux, uy))};
    double targetX {pose.x + stopAt * ux};
    double targetY {pose.y + stopAt * uy};
    std::uint32_t start {pros::millis()};
    while(pros::millis() - start < static_cast<std::uint32_t>(timeoutMs)){
        lemlib::Pose now {chassis.getPose()};
        if((targetX - now.x) * ux + (targetY - now.y) * uy <= 0.2){
            break;
        }
        //theta is clockwise positive, so a positive error steers right: left side faster
        double steer {std::clamp(1.5 * lemlib::angleError(heading, now.theta, false), -8.0, 8.0)};
        chassis.tank(cfg::pickupSpeed + steer, cfg::pickupSpeed - steer, true);
        pros::delay(10);
    }
    chassis.tank(0, 0, true);

    while(clawReopening){    //normally long done; closing first would get undone by its openClaw()
        pros::delay(10);
    }
    endEffector.closeClaw(closeExtraDeg);    //closeExtraDeg squeezes past the normal closed position
    if(clawFirst){
        pros::delay(cfg::clawMs);
        elevator.requestHeight(liftTo);
        waitElevatorNear(liftTo, config::elevator::resettleTolerance);    //off the ground before the robot backs out
        return;
    }
    elevator.requestHeight(liftTo);
    pros::delay(cfg::clawMs);    //the claw closes before the robot drives off
}

//turns to face a goal's center (x, y in the current frame) and drives to where the claw touches it, from wherever the
//robot is, so the aim and the run come from the pose instead of a fixed vector. the slow end is the same as the other goal runs
void Auton::driveToGoal(double goalX, double goalY){
    double heading {facePoint(goalX, goalY, 103)};
    lemlib::Pose pose {chassis.getPose()};
    double dx {goalX - pose.x};
    double dy {goalY - pose.y};
    double distance {std::hypot(dx, dy)};
    double run {distance - contactAt(heading)};
    driveSlowEnd(dx / distance * run, dy / distance * run, 4, 29, 45, 500);
}

//drives (dx, dy) from the current pose, fast until slowIn inches from the end, then drops hard to slowSpeed (0-127)
//open loop for the rest, holding the heading, and stops at the end point. not a moveToPoint with a maxSpeed for the end:
//LemLib raises that cap to at least 60 within 7.5 in of the target
void Auton::driveSlowEnd(double dx, double dy, double slowIn, int slowSpeed, int fastSpeed, int slowTimeoutMs){
    lemlib::Pose pose {chassis.getPose()};
    double distance {std::hypot(dx, dy)};
    double ux {dx / distance};
    double uy {dy / distance};
    double slowAt {distance - slowIn};
    //a raised elevator makes the robot sway, so the fast speed fades with the lift height (0 = down, 1 = top) and the
    //heading hold gets gentler (the same fractions as the driver control and the old push)
    double height {std::clamp(elevator.rotations() / config::elevator::maxMotorRotations, 0.0, 1.0)};
    fastSpeed = static_cast<int>(fastSpeed * (1.0 - (1.0 - config::drivetrain::elevatorTopSpeedScale) * height));
    double steerScale {1.0 - (1.0 - cfg::pushHeadingTopScale) * height};
    if(slowAt > 0){
        //hands over to the slow part within 1 in, without stopping
        chassis.moveToPoint(pose.x + slowAt * ux, pose.y + slowAt * uy, timeoutMs,
                            {.maxSpeed = static_cast<float>(fastSpeed), .minSpeed = static_cast<float>(slowSpeed), .earlyExitRange = 1}, false);
    }
    double heading {lemlib::radToDeg(std::atan2(ux, uy))};
    double targetX {pose.x + dx};
    double targetY {pose.y + dy};
    std::uint32_t start {pros::millis()};
    std::uint32_t movingAt {start};
    while(pros::millis() - start < static_cast<std::uint32_t>(slowTimeoutMs)){
        lemlib::Pose now {chassis.getPose()};
        //within 1 in of the end counts as there: against the goal the robot stalls short of the point and would wait out the timeout
        if((targetX - now.x) * ux + (targetY - now.y) * uy <= 1.0){
            break;
        }
        //stalled against the goal (same test as pushUntilStall): stop now instead of waiting out the timeout
        std::uint32_t t {pros::millis()};
        if(t - start < cfg::pushGraceMs || drivetrain.speedRpm() >= cfg::pushStallRpm){
            movingAt = t;
        }
        else if(t - movingAt >= cfg::pushStallConfirmMs){
            break;
        }
        //theta is clockwise positive, so a positive error steers right: left side faster
        double steer {std::clamp(1.5 * steerScale * lemlib::angleError(heading, now.theta, false), -8.0, 8.0)};
        chassis.tank(slowSpeed + steer, slowSpeed - steer, true);
        pros::delay(10);
    }
    chassis.tank(0, 0, true);
}

//score the preload on the left red Goal (the Station Goal), then the two cup+pin stacks and a Match Load on top of it.
//starts holding the preload against the red Alliance Station wall. after the first score() the goal is (0, 0)
void Auton::ogpreload(){
    chassis.setPose(0, 0, 0);
    endEffector.clampPreload();    //clamps the preload and zeroes the claw as it sets off, then the pivot goes down
    elevator.requestHeight(cfg::goalScoreRotations[0]);

    //1. preload: out along the wall to the goal's column, face the goal, score
    chassis.moveToPoint(0, 15, timeoutMs, {}, false);
    chassis.turnToHeading(90, timeoutMs, turn, false);
    score(0, 90);

    //the claw's zero is still shifted for the Pin: re-home it empty once clear of the Pin (closing any sooner could
    //grab it off the goal), then open it again, all in the background while the robot backs up
    endEffector.rehomeClaw(movedFromHere(cfg::rehomeClearance));
    clawReopening = true;
    pros::Task([this]{
        endEffector.waitForClawHome();
        endEffector.openClaw();
        clawReopening = false;
    });
    intakeWhenClear();
    backStraight(8 - cfg::alignBackoff, backThrough);    //score() already backed off alignBackoff

    //2. diagonal stack
    pickUp(1.715, 21.952, cfg::goalScoreRotations[1]);
    score(1, facePoint(0, 0));
    intakeWhenClear();
    backStraight(cfg::scoreClearIn, backThrough);

    //3. standing stack. the turn to the goal is slowed so the elevator reaches layer 2 before the push. TODO tune the 70
    pickUp(22.25, -1.78, cfg::goalScoreRotations[2]);
    score(2, facePoint(0, 0, 70));
    intakeWhenClear();
    backStraight(9, backThrough);

    //4. Match Load: grab it, back straight out of the Loader (TODO tune the 10 in), lift while turning to the goal
    pickUp(31.525, -20.855, config::elevator::intakeRotations + cfg::loadLiftRotations, true);
    backStraight(10, backThrough);
    elevator.requestHeight(cfg::goalScoreRotations[3]);
    score(3, facePoint(0, 0));
    backStraight(cfg::scoreClearIn, back);
}

//TODO: empty until the match routines are written
void Auton::matchLeft(){}

void Auton::matchRight(){}

void Auton::none(){}

//shows the pose on the brain screen for 120 s, so it can be read while the routine runs and after the robot stops. the
//last line is the heading furthest from 0 it reached, which shows how far a turn overshot
void Auton::showPose(){
    pros::Task([this]{
        std::uint32_t end {pros::millis() + 120000};
        double peak {0};
        while(pros::millis() < end){
            lemlib::Pose p {chassis.getPose()};
            if(std::abs(p.theta) > std::abs(peak)){
                peak = p.theta;
            }
            pros::screen::print(pros::E_TEXT_LARGE, 2, "x: %7.2f in", p.x);
            pros::screen::print(pros::E_TEXT_LARGE, 4, "y: %7.2f in", p.y);
            pros::screen::print(pros::E_TEXT_LARGE, 6, "heading: %7.2f", p.theta);
            pros::screen::print(pros::E_TEXT_LARGE, 8, "peak: %7.2f", peak);
            pros::delay(10);
        }
    });
}

//test: drive 48 in straight forward from the start pose, to check the odometry distance
void Auton::drive48(){
    chassis.setPose(0, 0, 0);
    showPose();
    chassis.moveToPoint(0, 48, 5000, {.maxSpeed = 80}, false);
    std::printf("drive48 ended at x %.2f y %.2f heading %.2f\n", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
}

//test: turn 90 deg in place from the start pose, to tune the turning. the screen shows the final and peak heading
void Auton::turn90(){
    chassis.setPose(0, 0, 0);
    showPose();
    chassis.turnToHeading(90, 4000, {}, false);
    std::printf("turn90 ended at heading %.2f\n", chassis.getPose().theta);
}

//the first one is the default if nobody touches the selector
const std::array<Auton::Routine, 7> Auton::routines{{
    {"Skills", &Auton::skills},
    {"Turn 90", &Auton::turn90},
    {"Drive 48", &Auton::drive48},
    {"OG Preload", &Auton::ogpreload},
    {"Match Left", &Auton::matchLeft},
    {"Match Right", &Auton::matchRight},
    {"None", &Auton::none},
}};

void Auton::select(){
    int shown {-1};
    while(true){
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT) != 0){
            selected = (selected + routines.size() - 1) % routines.size();
        }
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT) != 0){
            selected = (selected + 1) % routines.size();
        }
        //the controller drops prints sent faster than every 50 ms, so only print on a change
        if(selected != shown && master.print(0, 0, "Auto: %-12s", routines[selected].name) == 1){
            shown = selected;
        }
        pros::delay(50);
    }
}

bool Auton::homesElevatorWhileMoving() const{
    return routines[selected].run == &Auton::skills;
}

void Auton::run(){
    //the elevator only moves while update() runs, and opcontrol's loop isn't running. stops on its own when
    //autonomous (or the X + Y test) ends so it never runs alongside opcontrol's update()
    pros::Task([this]{
        while(pros::competition::is_connected()
              ? pros::competition::is_autonomous() && !pros::competition::is_disabled()
              : testing.load()){
            elevator.update();
            pros::delay(20);
        }
    });
    (this->*routines[selected].run)();
}
