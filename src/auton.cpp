#include "main.h"
#include "auton.h"
#include "drivetrain.h"
#include "elevator.h"
#include "endeffector.h"
#include "constants.h"
#include "api.h"
#include "lemlib/api.hpp"
#include <array>
#include <atomic>
#include <cmath>
#include <functional>

namespace auton{

    using drivetrain::chassis;

    //coordinates in inches relative to where the robot starts: its starting spot is (0, 0) facing heading 0, +y is
    //forward from the start, +x is to its right. LemLib headings: 0 along +y, clockwise positive. the spots were worked
    //out from the field figure (start at (8.5, 64.955) facing the blue Alliance Station, goals at (23.11, 46.66) and
    //(46.66, 23.11) from the red corner), so expect them to be off by an inch or two on a real field

    //LemLib's maxSpeed is 0-127 (TODO tune on the robot)
    static const lemlib::MoveToPointParams drive {.forwards = true, .maxSpeed = 127};
    static const lemlib::MoveToPointParams back {.forwards = false, .maxSpeed = 127};
    //backing out to a waypoint that is only there to reposition: hand over to the next turn within 2 in, still moving,
    //instead of stopping (the turn and pickup after it aim from wherever the robot ends up). TODO tune
    static const lemlib::MoveToPointParams backThrough {.forwards = false, .maxSpeed = 127, .minSpeed = 30, .earlyExitRange = 2};
    //every turn is followed by a move that steers or aligns itself (moveToPoint, or the push into the goal's aligner), so
    //turns hand over once within 3 deg instead of settling to 1 deg (minSpeed 1 only turns the early exit on)
    static const lemlib::TurnToHeadingParams turn {.maxSpeed = 127, .minSpeed = 1, .earlyExitRange = 3};
    static const lemlib::TurnToPointParams aim {.maxSpeed = 127, .minSpeed = 1, .earlyExitRange = 3};

    constexpr int timeoutMs {constants::autonPathTimeoutMs};

    //watch a turn that was just started (async) toward `target` deg and print how it went, for tuning angularController:
    //the size of the turn, how long it took, the error and overshoot when it handed over, its peak speed, and the heading
    //200 ms later (how far it kept rotating after the handover). blocks until the turn ends
    static void watchTurn(const char* label, double target){
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
        pros::Task([label, target]{
            pros::delay(200);
            std::printf("turn %s: error %.2f deg 200 ms after handover\n", label,
                        std::remainder(target - chassis.getPose().theta, 360.0));
        });
    }

    //turnToHeading / turnToPoint with the turn printed (see watchTurn). label names the turn in the printout. they
    //return the target heading, for score() to hold through the push
    static double faceHeading(const char* label, double heading){
        chassis.turnToHeading(heading, timeoutMs, turn, true);
        watchTurn(label, heading);
        return heading;
    }

    //maxSpeed (0-127) slows just this turn, e.g. to give the elevator time to rise before the push
    static double facePoint(const char* label, double x, double y, float maxSpeed = 127){
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
    static void waitElevatorNear(double rotations, double toleranceRotations, std::uint32_t timeoutMs = 1000){
        std::uint32_t start {pros::millis()};
        while(std::abs(elevator::getElevatorRotations() - rotations) > toleranceRotations && pros::millis() - start < timeoutMs){
            pros::delay(10);
        }
    }

    //set the Pin down on the goal (which already has `layer` Pins on it) and let go
    static void release(int layer){
        //it has been rising since the pickup, all through the push
        waitElevatorNear(constants::autonGoalScoreRotations[layer], constants::autonScoreDropToleranceRotations);
        double dropTo {constants::autonGoalScoreRotations[layer] - constants::autonScoreDropRotations};
        elevator::requestHeight(dropTo);
        waitElevatorNear(dropTo, constants::autonScoreDropToleranceRotations);
        endeffector::waitForClawHome();    //the preload home finishes on the way here
        endeffector::openClaw();
        pros::delay(constants::autonClawOpenMs);
        //no lift back up: the open claw slides off the Pin as the robot backs straight away, and intakeWhenClear() lowers
        //it from here
    }

    //drive toward (x, y) until the claw is over it: full speed until autonPickupSlowIn short of that, then the rest at
    //autonPickupSpeed. moveToPoint steers onto the line, so the turn before it only has to be close
    static void driveClawOver(double x, double y){
        lemlib::Pose pose {chassis.getPose()};
        double distance {std::hypot(x - pose.x, y - pose.y)};
        double ux {(x - pose.x) / distance};
        double uy {(y - pose.y) / distance};
        double stopAt {distance - constants::autonClawReach};
        double slowAt {stopAt - constants::autonPickupSlowIn};
        if(slowAt > 0){
            //minSpeed + earlyExitRange: hand over to the slow part within 1 in, without stopping
            chassis.moveToPoint(pose.x + slowAt * ux, pose.y + slowAt * uy, timeoutMs,
                                {.maxSpeed = 127, .minSpeed = constants::autonPickupSpeed, .earlyExitRange = 1}, false);
        }
        //hand over within 1 in instead of settling (friction can hold it just short for the 500 ms large exit), then stop
        chassis.moveToPoint(pose.x + stopAt * ux, pose.y + stopAt * uy, timeoutMs,
                            {.maxSpeed = constants::autonPickupSpeed, .minSpeed = 1, .earlyExitRange = 1}, false);
        chassis.tank(0, 0, true);
    }

    //tracking center to the goal's center once stalled against it: score() backs off autonAlignBackoff so the claw is
    //over the goal, so the robot must have stopped that far past the claw-over spot
    constexpr double contactIn {constants::autonClawReach - constants::autonAlignBackoff};

    //facing the goal (heading, from the turn before): once the elevator has the Pin near the scoring height, drive ahead
    //slowly holding heading until the drive motors stall against it, then reset the pose so the goal's center is (0, 0)
    //(the robot is contactIn behind it along the heading it ended on), back off to where the claw is over the goal, and
    //drop the Pin. everything after it is relative to the goal
    static void score(int layer, double heading){
        //push straight away, without waiting on the elevator: it keeps rising the whole way in, and release() waits for
        //it to reach the scoring height before dropping. the push's gentle hold fixes what the turn's handover left
        drivetrain::pushUntilStall(constants::autonPushSpeed, constants::autonPushTimeoutMs, heading);
        lemlib::Pose stalled {chassis.getPose()};
        double stalledAt {stalled.theta};
        double rad {lemlib::degToRad(stalledAt)};
        //where odometry thinks the goal is, from the pose it stalled at. after the first score this should be (0, 0): the
        //gap is how far odometry drifted (or the robot stalled somewhere else) since the last reset
        double foundX {stalled.x + contactIn * std::sin(rad)};
        double foundY {stalled.y + contactIn * std::cos(rad)};
        std::printf("score %d: found the goal at (%.2f, %.2f), %.2f in from where odometry had it, elevator at %.2f of %.2f\n",
                    layer, foundX, foundY, std::hypot(foundX, foundY), elevator::getElevatorRotations(),
                    constants::autonGoalScoreRotations[layer]);
        chassis.setPose(-contactIn * std::sin(rad), -contactIn * std::cos(rad), stalledAt);
        double backTo {contactIn + constants::autonAlignBackoff};
        chassis.moveToPoint(-backTo * std::sin(rad), -backTo * std::cos(rad), timeoutMs, back, false);
        release(layer);
    }

    //back straight out along the current heading, so the claw doesn't swing into the Pin it just dropped
    static void backStraight(double inches, const lemlib::MoveToPointParams& params){
        lemlib::Pose pose {chassis.getPose()};
        double rad {lemlib::degToRad(pose.theta)};
        chassis.moveToPoint(pose.x - inches * std::sin(rad), pose.y - inches * std::cos(rad), timeoutMs, params, false);
    }

    //true from here until the robot has moved `inches` from where it is now
    static std::function<bool()> movedFromHere(double inches){
        lemlib::Pose from {chassis.getPose()};
        return [from, inches]{
            lemlib::Pose pose {chassis.getPose()};
            return std::hypot(pose.x - from.x, pose.y - from.y) >= inches;
        };
    }

    //true while the claw is re-homing after the preload and hasn't opened again yet
    static std::atomic<bool> clawReopening {false};

    //after dropping the preload, the claw's zero is still shifted for the Pin. now it's empty and open, so it homes
    //properly on the move, once the robot is clear of the Pin (closing any sooner could grab it off the goal), then opens
    //again in the background so the robot never stops for it
    static void rehomeClawWhenClear(){
        endeffector::rehomeClaw(movedFromHere(constants::autonRehomeClearance));
        clawReopening = true;
        pros::Task([]{
            endeffector::waitForClawHome();
            endeffector::openClaw();
            clawReopening = false;
        });
    }

    //lower the elevator to the intake height once the robot has backed autonIntakeClearance from the goal (any sooner
    //and the claw could catch the Pin), in the background so it is down by the time the robot reaches the next stack
    static void intakeWhenClear(){
        pros::Task([clear = movedFromHere(constants::autonIntakeClearance)]{
            std::uint32_t start {pros::millis()};
            while(!clear() && pros::millis() - start < 3000){
                pros::delay(10);
            }
            elevator::requestIntake();
        });
    }

    //turn to face (x, y) (printed as `label`), close the claw on it from autonClawReach away, and lift it to liftTo (motor rotations).
    //clawFirst: let the claw close before the elevator moves (in the Loader, where lifting early can jam the claw);
    //otherwise the lift starts as the claw closes, so it rises while the robot turns and drives to the goal
    static void pickUp(const char* label, double x, double y, double liftTo, bool clawFirst = false){
        facePoint(label, x, y);
        //the claw has to be down at the intake height before it drives in. coming down from a high score can take longer
        //than the back-up and turn, so stop and wait if it isn't there yet (near is enough, no need for the full settle)
        if(std::abs(elevator::getElevatorRotations() - constants::elevatorIntakeRotations) > constants::elevatorResettleTolerance){
            chassis.tank(0, 0, true);
            waitElevatorNear(constants::elevatorIntakeRotations, constants::elevatorResettleTolerance, 2000);
        }
        driveClawOver(x, y);
        while(clawReopening){    //normally long done; closing first would get undone by its openClaw()
            pros::delay(10);
        }
        lemlib::Pose closedAt {chassis.getPose()};
        std::printf("pickup %s: claw closed with the robot at (%.2f, %.2f), heading %.1f deg\n", label, closedAt.x,
                    closedAt.y, closedAt.theta);
        endeffector::closeClaw();
        if(clawFirst){
            pros::delay(constants::autonClawMs);
            elevator::requestHeight(liftTo);
            waitElevatorNear(liftTo, constants::elevatorResettleTolerance);    //off the ground before the robot backs out
            return;
        }
        elevator::requestHeight(liftTo);
        pros::delay(constants::autonClawMs);    //the claw closes before the robot drives off
    }

    //test: score the preload on the left red Goal (the Station Goal), then the cup+pin stacks on top of it.
    //starts holding the preload against the red Alliance Station wall. in the start frame the goal is at (18.30, 14.61),
    //the diagonal stack at (18.30, 38.16) and the standing stack at (41.85, 14.61); after the first score() the goal is
    //(0, 0), so the diagonal stack is at (0, 23.55) and the standing one at (23.55, 0)
    static void preloadTest(){
        chassis.setPose(0, 0, 0);
        //clamp the preload and zero the claw as it sets off, then the pivot goes down
        endeffector::clampPreload();
        elevator::requestHeight(constants::autonGoalScoreRotations[0]);

        //out along the wall to the goal's column, face the goal, and push into it
        chassis.moveToPoint(0, 15, timeoutMs, drive, false);
        score(0, faceHeading("to the preload goal", 90));

        //back up along the wall to 8 in behind where it stalled, rolling straight into the turn toward the stack; the claw
        //re-homes and reopens and the elevator lowers on the way
        rehomeClawWhenClear();
        intakeWhenClear();
        backStraight(8 - constants::autonAlignBackoff, backThrough);    //score() already backed off autonAlignBackoff

        //pick up the diagonal stack (about 34 deg from there), then turn straight to the goal and score it on the preload
        pickUp("to the diagonal stack", 1.715, 21.952, constants::autonGoalScoreRotations[1]);
        score(1, facePoint("diagonal stack to the goal", 0, 0));

        //it scored from the goal's +y side and the standing stack is on its +x side, so go around: back out to where the
        //diagonal stack stood, which is on the 135 deg line through the standing stack and clear of both goals
        //(the claw is still open from the score). straight back first: curving away right off the goal swung the claw into
        //the Pin and knocked it over
        intakeWhenClear();
        backStraight(constants::autonScoreClearIn, backThrough);
        pickUp("to the standing stack", 22.25, -1.78, constants::autonGoalScoreRotations[2]);
        //slowed so the elevator has time to rise to layer 2 before the push. TODO tune the 70
        score(2, facePoint("standing stack to the goal", 0, 0, 70));

        intakeWhenClear();
        backStraight(9, backThrough);
        pickUp("to the Loader", 31.525, -20.855, constants::elevatorIntakeRotations + constants::autonLoadLiftRotations, true);

        //back straight out of the Loader, then lift to the scoring height while turning and driving to the goal, and score
        //on top of the three Pins. TODO tune the 10 in
        backStraight(10, backThrough);
        elevator::requestHeight(constants::autonGoalScoreRotations[3]);
        score(3, facePoint("Match Load to the goal", 0, 0));
        backStraight(constants::autonScoreClearIn, back);
    }

    static void matchLeft(){
        drivetrain::chassis.setPose(0, 0, 0);
        //TODO
    }

    static void matchRight(){
        drivetrain::chassis.setPose(0, 0, 0);
        //TODO
    }

    static void none(){}

    struct Routine{
        const char* name;
        void (*run)();
    };

    //the first one is the default if nobody touches the selector
    static constexpr std::array<Routine, 4> routines{{
        {"Preload Test", preloadTest},
        {"Match Left", matchLeft},
        {"Match Right", matchRight},
        {"None", none},
    }};
    static int selected {0};

    void select(){
        int shown {-1};
        while(true){
            if(constants::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT) != 0){
                selected = (selected + routines.size() - 1) % routines.size();
            }
            if(constants::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT) != 0){
                selected = (selected + 1) % routines.size();
            }
            //the controller drops prints sent faster than every 50 ms, so only print on a change
            if(selected != shown && constants::master.print(0, 0, "Auto: %-12s", routines[selected].name) == 1){
                shown = selected;
            }
            pros::delay(50);
        }
    }

    //an X + Y test run from driver control
    static bool testRunning {false};
    static pros::Task* testTask {nullptr};

    void run(){
        //the elevator only moves while update() runs, and opcontrol's loop isn't running. stops on its own when
        //autonomous (or the X + Y test) ends so it never runs alongside opcontrol's update()
        pros::Task([]{
            while(pros::competition::is_connected()
                  ? pros::competition::is_autonomous() && !pros::competition::is_disabled()
                  : testRunning){
                elevator::update();
                pros::delay(20);
            }
        });
        routines[selected].run();
    }

    bool driverTest(){
        auto& master {constants::master};
        if(!testRunning){
            if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X) != 0 && master.get_digital(pros::E_CONTROLLER_DIGITAL_Y) != 0){
                testRunning = true;
                //use up the presses so the elevator doesn't jump to the X / Y level once the run ends
                master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X);
                master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y);
                delete testTask;
                testTask = new pros::Task([]{
                    //always the preload home, like a real autonomous, even if X homed (empty) earlier
                    elevator::forgetHome();
                    endeffector::forgetHome();
                    autonomous();
                    testRunning = false;
                });
            }
            return testRunning;
        }

        //there is no competition switch to disable the robot, so B or either stick stops the run
        bool stop {master.get_digital(pros::E_CONTROLLER_DIGITAL_B) != 0
                   || std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) > 30
                   || std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) > 30};
        if(stop){
            testTask->remove();
            chassis.cancelAllMotions();
            testRunning = false;
        }
        return testRunning;
    }

}
