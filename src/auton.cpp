#include "main.h"
#include "auton.h"
#include "drivetrain.h"
#include "elevator.h"
#include "endeffector.h"
#include "constants.h"
#include "api.h"
#include "lemlib/api.hpp"
#include <array>
#include <cmath>

namespace auton{

    using drivetrain::chassis;

    //field coordinates in inches, origin at the field center, +x toward the blue Alliance Station, +y toward the top of
    //Figure VURS1 (the red drivers' left). LemLib headings: 0 along +y, clockwise positive. spots were read off the
    //figure, so expect them to be off by an inch or two on a real field
    struct Point{
        float x;
        float y;
    };
    //a real field is 6 tiles of 23.4" (140.4" inside the walls, not 144"), so the walls are at +-3 tiles = +-70.2"
    constexpr float tile {23.4f};
    constexpr float wall {3 * tile};
    //the robot is 15" square. assumes the tracking center is the middle of the robot, TODO adjust if it isn't
    constexpr float robotHalf {7.5f};

    //the game manual measures from the bottom-left corner (the field center is (70.2, 70.2)). this turns those numbers
    //into the code's center-origin coordinates
    constexpr Point fromManual(float x, float y){
        return {x - wall, y - wall};
    }

    //center of the lower red Loader's bottom opening, where the claw closes on a load: ~2" off the red Alliance
    //Station wall, and at y = -58.76
    constexpr Point loaderStack {-wall + 2, -58.76f};
    //robot spot straight out from the Loader. the robot turns to face the wall here, then drives straight in, so its
    //corners clear the bottom wall
    constexpr Point loaderFront {loaderStack.x + constants::autonClawReach + 4, loaderStack.y};

    //path.jerryio paths for the long legs. in jerryio use inches with the origin at the field center (the coordinates
    //above), export "LemLib v0.5", and save it as static/<name>.txt (any name but example.txt, which LemLib already
    //uses). then add ASSET(<name>_txt) here, with the dot as an underscore, and put &<name>_txt on its leg below.
    //a leg with no path (nullptr) drives straight to its end point instead
    //ASSET(stationToLoader_txt)

    //the two red Alliance Goals (3.25" tall), each with a spot the robot drives through between it and the Loader, and
    //its two path legs: lane to loaderFront, and loaderFront to lane. the lanes go around the upright object on the
    //diagonal tape at (-2, -2) tiles
    struct Goal{
        Point center;
        Point lane;
        const asset* toLoader;
        const asset* fromLoader;
    };
    //in the Quadrant next to the red Alliance Station. its lane is clear of the wall stack at (-wall, -1 tile)
    constexpr Goal stationGoal {{-2 * tile, -tile}, {-2.6f * tile, -1.6f * tile}, nullptr, nullptr};
    //in the Quadrant along the bottom wall. its lane runs below the diagonal object, 2.7" off the bottom wall
    constexpr Goal bottomGoal {{-tile, -2 * tile}, {-40, -wall + robotHalf + 2.7f}, nullptr, nullptr};

    static void waitElevator(std::uint32_t timeoutMs = 2000){
        std::uint32_t start {pros::millis()};
        while(!elevator::isSettled() && pros::millis() - start < timeoutMs){
            pros::delay(20);
        }
    }

    //turn toward p (or away from it, to back up)
    //returns the heading it faces (or would, if the turn was skipped). a turn under skipDeg isn't worth making: it only
    //twists the robot (e.g. back toward where it was before the aligner squared it); tolDeg > 0 loosens when a turn ends
    static double faceTo(Point p, bool forwards, double speedScale = 1.0, double skipDeg = constants::autonSkipTurnDeg,
                         double tolDeg = 0){
        lemlib::Pose pose {chassis.getPose()};
        double facing {lemlib::radToDeg(std::atan2(p.x - pose.x, p.y - pose.y))};
        double target {forwards ? facing : facing + 180};
        if(std::abs(std::remainder(target - pose.theta, 360.0)) > skipDeg){
            drivetrain::turnTo(target, speedScale, tolDeg);
        }
        return target;
    }

    //turn toward p (or away from it, to back up), then drive straight to it, stopping `shortBy` inches early
    static void driveTo(Point p, bool forwards = true, float shortBy = 0, double speedScale = 1.0,
                        double skipDeg = constants::autonSkipTurnDeg, double tolDeg = 0){
        faceTo(p, forwards, speedScale, skipDeg, tolDeg);
        lemlib::Pose pose {chassis.getPose()};
        double dist {std::hypot(p.x - pose.x, p.y - pose.y) - shortBy};
        if(dist > 0){
            drivetrain::driveStraight(forwards ? dist : -dist, speedScale);
        }
    }

    //driveTo for a move that only passes through the point (the next move re-aims from wherever this ends)
    static void passThrough(Point p, bool forwards = true){
        driveTo(p, forwards, 0, constants::autonFastScale, constants::autonLooseSkipTurnDeg, constants::autonLooseTurnTolDeg);
    }

    //one long leg: follow its path.jerryio path (blocks until done), or drive straight to `end` while it has none.
    //pure pursuit doesn't turn in place first, so draw each path starting the way the robot faces at its start
    static void travel(const asset* path, Point end){
        if(path == nullptr){
            driveTo(end);
            return;
        }
        chassis.follow(*path, constants::autonPathLookahead, constants::autonPathTimeoutMs, true, false);
    }

    //turn to face p, then drive until the claw is over it
    static void clawTo(Point p){
        driveTo(p, true, constants::autonClawReach);
    }

    //same, but fast until the last few inches, which run at the tuned speed. it's one motion: the speed eases down to the
    //slow one without stopping
    static void clawToFast(Point p){
        faceTo(p, true, constants::autonFastScale);
        lemlib::Pose pose {chassis.getPose()};
        double total {std::hypot(p.x - pose.x, p.y - pose.y) - constants::autonClawReach};
        if(total > 0){
            drivetrain::driveStraight(total, constants::autonFastScale, constants::autonPickupSlowIn);
        }
    }

    //drop what the claw holds onto the goal, which already has `layer` Pins on it, then (unless told not to) back out to
    //its lane
    //aimShift moves where the claw aims from the goal's center (inches, field axes), for correcting a measured miss
    static void scoreOn(const Goal& goal, int layer, bool toLane = true, Point aimShift = {0, 0}){
        elevator::requestHeight(constants::autonGoalScoreRotations[layer]);
        //one fluid motion: turn to face the goal, drive up to where the claw would be over it and carry straight on into the
        //aligner without stopping. the aligner centers the robot on the goal; backing off from there sets where the claw
        //ends up
        double toGoal {faceTo({goal.center.x + aimShift.x, goal.center.y + aimShift.y}, true, constants::autonFastScale,
                               constants::autonLooseSkipTurnDeg, constants::autonLooseTurnTolDeg)};
        lemlib::Pose pose {chassis.getPose()};
        double toClaw {std::hypot(goal.center.x + aimShift.x - pose.x, goal.center.y + aimShift.y - pose.y)
                       - constants::autonClawReach};
        drivetrain::driveInto(std::max(0.0, toClaw), constants::autonAlignMs, constants::autonFastScale, toGoal);
        drivetrain::driveStraight(-constants::autonAlignBackoff);
        waitElevator();    //it has been rising the whole way, so this is normally already done
        //the circular aligner centers the robot on the goal but doesn't square it, so the claw is now over the goal's
        //center along whatever heading the robot has: reset the position to that and keep the IMU's heading (snapping it to
        //an axis put the pickup legs several degrees off). the aim shift is added back in because the moves after this (the
        //stack pickups) were tuned in the frame where the claw's believed spot was the goal plus that shift
        lemlib::Pose backedOff {chassis.getPose()};
        double rad {lemlib::degToRad(backedOff.theta)};
        chassis.setPose(goal.center.x + aimShift.x - constants::autonClawReach * std::sin(rad),
                        goal.center.y + aimShift.y - constants::autonClawReach * std::cos(rad), backedOff.theta);
        //set the Pin down on the goal before letting go, instead of dropping it from the carry height
        double dropTo {constants::autonGoalScoreRotations[layer] - constants::autonScoreDropRotations};
        elevator::requestHeight(dropTo);
        //open as soon as it's nearly down, instead of waiting for it to fully settle
        std::uint32_t dropStart {pros::millis()};
        while(std::abs(elevator::getElevatorRotations() - dropTo) > constants::autonScoreDropToleranceRotations
              && pros::millis() - dropStart < 1000){
            pros::delay(10);
        }
        endeffector::waitForClawHome();    //the preload home finishes on the way here
        endeffector::openClaw();
        pros::delay(constants::autonClawMs);
        if(toLane){
            driveTo(goal.lane, false);
        }
    }

    //after dropping the preload, the claw's zero is still shifted for the Pin. now it's empty and open, so it homes
    //properly on the move, once the robot is clear of the Pin (closing any sooner could grab it off the goal)
    static void rehomeClawWhenClear(){
        lemlib::Pose dropped {chassis.getPose()};
        endeffector::rehomeClaw([dropped]{
            lemlib::Pose pose {chassis.getPose()};
            return std::hypot(pose.x - dropped.x, pose.y - dropped.y) >= constants::autonRehomeClearance;
        });
    }

    //take the next Match Load out of the Loader and score it on the goal. a lone Pin (the first load of an empty goal)
    //is picked up from the intake height with the pivot angled down
    static const Goal* lastGoal {nullptr};

    static void loadTo(const Goal& goal, int layer, bool lonePin = false){
        endeffector::openClaw();
        elevator::requestIntake();
        if(lonePin){
            endeffector::pivotTo(constants::endEffectorPivotPinDeg);
        }
        //from the lane of the goal it last scored on
        travel(lastGoal->toLoader, loaderFront);
        drivetrain::turnTo(270);
        drivetrain::driveStraight(loaderFront.x - (loaderStack.x + constants::autonClawReach));
        waitElevator();
        endeffector::closeClaw();
        pros::delay(constants::autonClawMs);
        if(lonePin){
            endeffector::pivotTo(constants::endEffectorPivotHorizontalDeg);
        }
        elevator::requestHeight(constants::autonGoalScoreRotations[layer]);
        drivetrain::driveStraight(-(loaderFront.x - (loaderStack.x + constants::autonClawReach)));
        travel(goal.fromLoader, goal.lane);
        lastGoal = &goal;
        scoreOn(goal, layer);
    }

    //builds both red Alliance Goals up to 4 Pins each from the lower red Loader, alternating so a short run still
    //scores on both. the preload Pin goes on the Station Goal first, then the drive team loads: stack, lone Pin, then
    //stacks. each Pin goes in with the half it should score on top (red scores 5 now, yellow scores 10 once that
    //Quadrant's Toggle is red).
    //starts holding the preload, with its back against the red Alliance Station wall, halfway between the wall stack
    //(1 tile down) and the Load Zone tape (2 tiles down), facing into the field
    static void skills(){
        chassis.setPose(-wall + robotHalf, -1.5f * tile, 90);
        //the elevator and pivot are homed by now. clamp the preload and zero the claw as it sets off, then the pivot goes down
        endeffector::clampPreload();
        elevator::requestHeight(constants::autonGoalScoreRotations[0]);
        //straight out along the wall to the goal's column, then scoreOn turns to face it and drives in
        drivetrain::driveStraight(stationGoal.center.x - (-wall + robotHalf));
        scoreOn(stationGoal, 0, false);
        //back to the lane with the claw closing all the way and zeroing on the move
        rehomeClawWhenClear();
        driveTo(stationGoal.lane, false);
        endeffector::waitForClawHome();
        lastGoal = &stationGoal;
        loadTo(stationGoal, 1);
        loadTo(bottomGoal, 0, true);
        for(int layer {1}; layer <= 3; ++layer){
            loadTo(bottomGoal, layer);
            if(layer < 3){
                loadTo(stationGoal, layer + 1);
            }
        }
    }

    //where the robot backs up to after the preload, and the standing cup+pin stack on the diagonal tape
    constexpr Point afterPreload {fromManual(23.11f, 70.20f)};
    constexpr Point diagonalStack {fromManual(46.66f, 46.66f)};
    constexpr Point standingStack {fromManual(23.11f, 23.11f)};
    //where the claw aims to pick the standing stack up: this many inches further along the approach (the robot comes in at
    //225 deg, so -x and -y), because it kept stopping short. TODO tune: raise it until the claw closes around the center
    constexpr float standingStackPastIn {0.0f};
    constexpr Point standingStackAim {standingStack.x - 0.707f * standingStackPastIn,
                                      standingStack.y - 0.707f * standingStackPastIn};

    //per-score aim corrections (inches, field axes), the negative of the miss measured on the field. the start pose and the
    //claw reach are shared by every move, so a fix that centers one score moved the others; these only move their own.
    //preload: it landed 0.75 in +x and 0.75 in +y (plus 0.25 in +y from the 0.25 longer reach). second score: 2 in +y, and
    //0.25 short in x from the longer reach. then with (-0.25, -2.0) it landed 2.68 in -y and 2.1 in +x (short), so this
    //is that shift minus the miss. TODO tune
    //with the aligner these no longer move the real claw: scoreOn resets the pose to the goal plus this shift, which keeps
    //the believed frame the later moves were tuned in
    constexpr Point preloadAimShift {-0.75f, -1.0f};
    //zero now: the old (-2.35, 0.68) patched the miss from odometry drift before the aligner. scoreOn adds the shift to the
    //pose it resets to, so with it the robot believed it was 2.35 in x / 0.68 in y from where it was, and the next pickup
    //(the standing stack) came up short. the aligner makes the real position exact, so no patch is needed. TODO tune
    constexpr Point secondAimShift {0.0f, 0.0f};

    //test: score the preload on the left red Goal (the Station Goal), then the cup+pin stack on top of it.
    //starts holding the preload against the red Alliance Station wall, its center about 7.5" off the wall and 7.75" below
    //y = 0
    static void preloadTest(){
        //measured 7" and 7.25", but the robot kept reaching the goal 0.5" too far +x and -y, so the real start is shifted
        //by that (TODO confirm with a tape measure after the first leg)
        constexpr float startX {-wall + 7.5f};
        //y and the claw reach were tuned together from the stack and second-goal misses: the first goal can't tell them
        //apart (it's the same line as the reach), the stack and the second goal approach from other directions and can
        chassis.setPose(startX, -5.25f, 90);
        //clamp the preload and zero the claw as it sets off, then the pivot goes down
        endeffector::clampPreload();
        elevator::requestHeight(constants::autonGoalScoreRotations[0]);
        //out along the wall to above the goal (x = -46.67), then scoreOn turns to face it and drives in until the claw is over it
        drivetrain::driveStraight(-46.67 - startX, constants::autonFastScale);
        scoreOn(stationGoal, 0, false, preloadAimShift);

        rehomeClawWhenClear();
        passThrough(afterPreload, false);

        //pick up the stack: turn to it (about 45 deg) and drive in until the claw is over it
        elevator::requestIntake();
        endeffector::waitForClawHome();
        endeffector::openClaw();
        clawToFast(diagonalStack);
        waitElevator();
        endeffector::closeClaw();
        pros::delay(constants::autonClawMs);
        elevator::requestHeight(constants::autonGoalScoreRotations[1]);

        //carry it to where it stood, face -x, and score it on the preload
        passThrough(diagonalStack);
        drivetrain::turnTo(270, constants::autonFastScale, constants::autonLooseTurnTolDeg);
        scoreOn(stationGoal, 1, false, secondAimShift);

        //back to where the first stack stood, then to the next standing stack on the diagonal (straight down-left from there)
        passThrough(diagonalStack, false);
        elevator::requestIntake();
        endeffector::openClaw();
        clawToFast(standingStackAim);
        waitElevator();
        endeffector::closeClaw();
        pros::delay(constants::autonClawMs);
        elevator::requestHeight(constants::autonGoalScoreRotations[2]);

        //carry it to where it stood, face the goal (+y), and score it on the two Pins there
        passThrough(standingStack);
        drivetrain::turnTo(0, constants::autonFastScale, constants::autonLooseTurnTolDeg);
        scoreOn(stationGoal, 2, false);
        passThrough(standingStack, false);
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
    static constexpr std::array<Routine, 5> routines{{
        {"Preload Test", preloadTest},
        {"Skills", skills},
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
