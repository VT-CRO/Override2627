#include "auton/auton.h"
#include "config/auton.h"
#include "config/elevator.h"
#include <array>
#include <atomic>
#include <cmath>
#include <functional>


namespace cfg = config::auton;

Auton::Auton(pros::v5::Controller& master, Drivetrain& drivetrain, Elevator& elevator, EndEffector& endEffector)
    : master(master), drivetrain(drivetrain), chassis(drivetrain.chassis), elevator(elevator), endEffector(endEffector) {}

namespace {
//coordinates in inches relative to where the robot starts: its starting spot is (0, 0) facing heading 0, +y is
//forward from the start, +x is to its right. LemLib headings: 0 along +y, clockwise positive. the spots were worked
//out from the field figure (start at (8.5, 64.955) facing the blue Alliance Station, goals at (23.11, 46.66) and
//(46.66, 23.11) from the red corner), so expect them to be off by an inch or two on a real field

//LemLib's maxSpeed is 0-127 (TODO tune on the robot)
const lemlib::MoveToPointParams drive {.forwards = true, .maxSpeed = 127};
const lemlib::MoveToPointParams back {.forwards = false, .maxSpeed = 127};
//backing out to a waypoint that is only there to reposition: hand over to the next turn within 2 in, still moving,
//instead of stopping (the turn and pickup after it aim from wherever the robot ends up). TODO tune
const lemlib::MoveToPointParams backThrough {.forwards = false, .maxSpeed = 127, .minSpeed = 30, .earlyExitRange = 2};
//every turn is followed by a move that steers or aligns itself (moveToPoint, or the push into the goal's aligner), so
//turns hand over once within 3 deg instead of settling to 1 deg (minSpeed 1 only turns the early exit on)
const lemlib::TurnToHeadingParams turn {.maxSpeed = 127, .minSpeed = 1, .earlyExitRange = 3};
const lemlib::TurnToPointParams aim {.maxSpeed = 127, .minSpeed = 1, .earlyExitRange = 3};

constexpr int timeoutMs {cfg::pathTimeoutMs};

}

//watch a turn that was just started (async) toward `target` deg and print how it went, for tuning angularController:
//the size of the turn, how long it took, the error and overshoot when it handed over, its peak speed, and the heading
//200 ms later (how far it kept rotating after the handover). blocks until the turn ends
void Auton::watchTurn(const char* label, double target){
    double start {chassis.getPose().theta};
    double degrees {std::remainder(target - start, 360.0)};
    double sign {degrees < 0 ? -1.0 : 1.0};
    std::uint32_t startMs {pros::millis()};
    double overshoot {0};
    double peakSpeed {0};
    double last {start};
    pros::delay(10);    //let the async motion start before watching isInMotion
    while(chassis.isInMotion()){
        double theta {chassis.getPose().theta};
        overshoot = std::max(overshoot, std::remainder(theta - target, 360.0) * sign);
        peakSpeed = std::max(peakSpeed, std::abs(theta - last) * 100);    //10 ms loop, so deg/s
        last = theta;
        pros::delay(10);
    }
    double end {chassis.getPose().theta};
    std::printf("turn %s: %.1f deg in %lu ms, error %.2f deg at handover, overshoot %.2f deg, peak %.0f deg/s\n",
                label, degrees, static_cast<unsigned long>(pros::millis() - startMs),
                std::remainder(target - end, 360.0), overshoot, peakSpeed);
    pros::Task([this, label, target]{
        pros::delay(200);
        std::printf("turn %s: error %.2f deg 200 ms after handover\n", label,
                    std::remainder(target - chassis.getPose().theta, 360.0));
    });
}

//turnToPoint with the turn printed (see watchTurn). label names the turn in the printout. returns the heading the robot ended
//aimed along, for score() to hold through the push. maxSpeed (0-127) slows just this turn, e.g. to give the elevator time to rise before the push
double Auton::facePoint(const char* label, double x, double y, float maxSpeed){
    lemlib::Pose pose {chassis.getPose()};
    double heading {lemlib::radToDeg(std::atan2(x - pose.x, y - pose.y))};
    lemlib::TurnToPointParams params {aim};
    params.maxSpeed = maxSpeed;
    chassis.turnToPoint(x, y, timeoutMs, params, true);
    watchTurn(label, heading);
    //aim from where the turn ended, not where it started: after a backThrough the robot is still rolling as it turns,
    //and the start's heading sent the push clockwise of the Match Load's goal
    lemlib::Pose end {chassis.getPose()};
    return lemlib::radToDeg(std::atan2(x - end.x, y - end.y));
}

//wait until the elevator is within toleranceRotations of `rotations`, without waiting for it to fully settle
void Auton::waitElevatorNear(double rotations, double toleranceRotations, std::uint32_t timeoutMs){
    std::uint32_t start {pros::millis()};
    while(std::abs(elevator.rotations() - rotations) > toleranceRotations && pros::millis() - start < timeoutMs){
        pros::delay(10);
    }
}

//set the Pin down on the goal (which already has `layer` Pins on it) and let go
void Auton::release(int layer){
    //it has been rising since the pickup, all through the push
    waitElevatorNear(cfg::goalScoreRotations[layer], cfg::scoreDropToleranceRotations);
    double dropTo {cfg::goalScoreRotations[layer] - cfg::scoreDropRotations};
    elevator.requestHeight(dropTo);
    waitElevatorNear(dropTo, cfg::scoreDropToleranceRotations);
    endEffector.waitForClawHome();    //the preload home finishes on the way here
    endEffector.openClaw();
    pros::delay(cfg::clawOpenMs);
    //no lift back up: the open claw slides off the Pin as the robot backs straight away, and intakeWhenClear() lowers
    //it from here
}

//tracking center to the goal's center once stalled against it: score() backs off alignBackoff so the claw is
//over the goal, so the robot must have stopped that far past the claw-over spot
constexpr double contactIn {cfg::clawReach - cfg::alignBackoff};

//facing the goal (heading, from the turn before): once the elevator has the Pin near the scoring height, drive ahead
//slowly holding heading until the drive motors stall against it, then reset the pose so the goal's center is (0, 0)
//(the robot is contactIn behind it along the heading it ended on), back off to where the claw is over the goal, and
//drop the Pin. everything after it is relative to the goal
void Auton::score(int layer, double heading){
    //push straight away, without waiting on the elevator: it keeps rising the whole way in, and release() waits for
    //it to reach the scoring height before dropping. the push's gentle hold fixes what the turn's handover left
    drivetrain.pushUntilStall(cfg::pushSpeed, cfg::pushTimeoutMs, heading);
    lemlib::Pose stalled {chassis.getPose()};
    double stalledAt {stalled.theta};
    double rad {lemlib::degToRad(stalledAt)};
    //where odometry thinks the goal is, from the pose it stalled at. after the first score this should be (0, 0): the
    //gap is how far odometry drifted (or the robot stalled somewhere else) since the last reset
    double foundX {stalled.x + contactIn * std::sin(rad)};
    double foundY {stalled.y + contactIn * std::cos(rad)};
    std::printf("score %d: found the goal at (%.2f, %.2f), %.2f in from where odometry had it, elevator at %.2f of %.2f\n",
                layer, foundX, foundY, std::hypot(foundX, foundY), elevator.rotations(),
                cfg::goalScoreRotations[layer]);
    chassis.setPose(-contactIn * std::sin(rad), -contactIn * std::cos(rad), stalledAt);
    double backTo {contactIn + cfg::alignBackoff};
    chassis.moveToPoint(-backTo * std::sin(rad), -backTo * std::cos(rad), timeoutMs, back, false);
    release(layer);
}

//back straight out along the current heading, so the claw doesn't swing into the Pin it just dropped
void Auton::backStraight(double inches, const lemlib::MoveToPointParams& params){
    lemlib::Pose pose {chassis.getPose()};
    double rad {lemlib::degToRad(pose.theta)};
    chassis.moveToPoint(pose.x - inches * std::sin(rad), pose.y - inches * std::cos(rad), timeoutMs, params, false);
}

//true from here until the robot has moved `inches` from where it is now
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

//turn to face (x, y) (printed as `label`), close the claw on it from clawReach away, and lift it to liftTo (motor rotations).
//clawFirst: let the claw close before the elevator moves (in the Loader, where lifting early can jam the claw);
//otherwise the lift starts as the claw closes, so it rises while the robot turns and drives to the goal
void Auton::pickUp(const char* label, double x, double y, double liftTo, bool clawFirst){
    facePoint(label, x, y);
    //the claw has to be down at the intake height before it drives in. coming down from a high score can take longer
    //than the back-up and turn, so stop and wait if it isn't there yet (near is enough, no need for the full settle)
    if(std::abs(elevator.rotations() - config::elevator::intakeRotations) > config::elevator::resettleTolerance){
        chassis.tank(0, 0, true);
        waitElevatorNear(config::elevator::intakeRotations, config::elevator::resettleTolerance, 2000);
    }
        //drive toward (x, y) until the claw is over it: full speed until pickupSlowIn short of that, then the rest at
    //pickupSpeed. moveToPoint steers onto the line, so the turn before it only has to be close
    lemlib::Pose pose {chassis.getPose()};
        double distance {std::hypot(x - pose.x, y - pose.y)};
        double ux {(x - pose.x) / distance};
        double uy {(y - pose.y) / distance};
        double stopAt {distance - cfg::clawReach};
        double slowAt {stopAt - cfg::pickupSlowIn};
        if(slowAt > 0){
            //minSpeed + earlyExitRange: hand over to the slow part within 1 in, without stopping
            chassis.moveToPoint(pose.x + slowAt * ux, pose.y + slowAt * uy, timeoutMs,
                                {.maxSpeed = 127, .minSpeed = cfg::pickupSpeed, .earlyExitRange = 1}, false);
        }
        //hand over within 1 in instead of settling (friction can hold it just short for the 500 ms large exit), then stop
        chassis.moveToPoint(pose.x + stopAt * ux, pose.y + stopAt * uy, timeoutMs,
                            {.maxSpeed = cfg::pickupSpeed, .minSpeed = 1, .earlyExitRange = 1}, false);
        chassis.tank(0, 0, true);
    while(clawReopening){    //normally long done; closing first would get undone by its openClaw()
        pros::delay(10);
    }
    lemlib::Pose closedAt {chassis.getPose()};
    std::printf("pickup %s: claw closed with the robot at (%.2f, %.2f), heading %.1f deg\n", label, closedAt.x,
                closedAt.y, closedAt.theta);
    endEffector.closeClaw();
    if(clawFirst){
        pros::delay(cfg::clawMs);
        elevator.requestHeight(liftTo);
        waitElevatorNear(liftTo, config::elevator::resettleTolerance);    //off the ground before the robot backs out
        return;
    }
    elevator.requestHeight(liftTo);
    pros::delay(cfg::clawMs);    //the claw closes before the robot drives off
}

//test: score the preload on the left red Goal (the Station Goal), then the cup+pin stacks on top of it.
//starts holding the preload against the red Alliance Station wall. in the start frame the goal is at (18.30, 14.61),
//the diagonal stack at (18.30, 38.16) and the standing stack at (41.85, 14.61); after the first score() the goal is
//(0, 0), so the diagonal stack is at (0, 23.55) and the standing one at (23.55, 0)
void Auton::preloadTest(){
    chassis.setPose(0, 0, 0);
    //clamp the preload and zero the claw as it sets off, then the pivot goes down
    endEffector.clampPreload();
    elevator.requestHeight(cfg::goalScoreRotations[0]);

    //out along the wall to the goal's column, face the goal, and push into it
    chassis.moveToPoint(0, 15, timeoutMs, drive, false);
    chassis.turnToHeading(90, timeoutMs, turn, true);
    watchTurn("to the preload goal", 90);
    score(0, 90);

    //back up along the wall to 8 in behind where it stalled, rolling straight into the turn toward the stack; the claw
    //re-homes and reopens and the elevator lowers on the way
    //the claw's zero is still shifted for the Pin: now it's empty and open, so it homes properly on the move, once the
    //robot is clear of the Pin (closing any sooner could grab it off the goal), then opens again in the background
    endEffector.rehomeClaw(movedFromHere(cfg::rehomeClearance));
    clawReopening = true;
    pros::Task([this]{
        endEffector.waitForClawHome();
        endEffector.openClaw();
        clawReopening = false;
    });
    intakeWhenClear();
    backStraight(8 - cfg::alignBackoff, backThrough);    //score() already backed off alignBackoff

    //pick up the diagonal stack (about 34 deg from there), then turn straight to the goal and score it on the preload
    pickUp("to the diagonal stack", 1.715, 21.952, cfg::goalScoreRotations[1]);
    score(1, facePoint("diagonal stack to the goal", 0, 0));

    //it scored from the goal's +y side and the standing stack is on its +x side, so go around: back out to where the
    //diagonal stack stood, which is on the 135 deg line through the standing stack and clear of both goals
    //(the claw is still open from the score). straight back first: curving away right off the goal swung the claw into
    //the Pin and knocked it over
    intakeWhenClear();
    backStraight(cfg::scoreClearIn, backThrough);
    pickUp("to the standing stack", 22.25, -1.78, cfg::goalScoreRotations[2]);
    //slowed so the elevator has time to rise to layer 2 before the push. TODO tune the 70
    score(2, facePoint("standing stack to the goal", 0, 0, 70));

    intakeWhenClear();
    backStraight(9, backThrough);
    pickUp("to the Loader", 31.525, -20.855, config::elevator::intakeRotations + cfg::loadLiftRotations, true);

    //back straight out of the Loader, then lift to the scoring height while turning and driving to the goal, and score
    //on top of the three Pins. TODO tune the 10 in
    backStraight(10, backThrough);
    elevator.requestHeight(cfg::goalScoreRotations[3]);
    score(3, facePoint("Match Load to the goal", 0, 0));
    backStraight(cfg::scoreClearIn, back);
}

void Auton::matchLeft(){
    chassis.setPose(0, 0, 0);
    //TODO
}

void Auton::matchRight(){
    chassis.setPose(0, 0, 0);
    //TODO
}

void Auton::none(){}

//the first one is the default if nobody touches the selector
const std::array<Auton::Routine, 4> Auton::routines{{
    {"Preload Test", &Auton::preloadTest},
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
