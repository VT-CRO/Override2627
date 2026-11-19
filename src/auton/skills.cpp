#include "auton/auton.h"
#include "config/auton.h"
#include "config/endeffector.h"
#include <cstdio>

namespace cfg = config::auton;

namespace {
constexpr int timeoutMs {cfg::pathTimeoutMs};
}

//skills run. local coordinates: the start is (0, 0) facing heading 0, +y is forward, +x is right.
//the routine scores 6 Pins. each PIN section below says where it is picked up and where it is scored, and the step
//numbers run through the whole routine, so a Pin number and a step number find the same code
void Auton::skills(){
    chassis.setPose(0, 0, 0);
    //timing: shows how long each step took (ms since the last lap) in a column on the brain screen
    std::uint32_t lapStart {pros::millis()};
    int lapRow {0};
    auto lap {[&](const char* name){
        std::uint32_t now {pros::millis()};
        pros::screen::print(pros::E_TEXT_MEDIUM, 20, 8 + 24 * lapRow, "%s: %u ms", name, static_cast<unsigned>(now - lapStart));
        std::printf("lap %s %u ms\n", name, static_cast<unsigned>(now - lapStart));
        lapStart = now;
        ++lapRow;
    }};
    endEffector.clampPreload();
    elevator.requestHeight(cfg::goalScoreRotations[0]);

    //======== PIN 1: the preload, scored on goal 1 at layer 0 ========
    //step 1: drive 2 in forward, turn to -54
    chassis.moveToPoint(0, 4, 2500, {.maxSpeed = 103, .earlyExitRange = 1}, false);    //exits within 1 in: it stops short and would wait out the 500 ms hold
    chassis.tank(0, 0, true);
    chassis.turnToHeading(-54, 2000, {.maxSpeed = 103}, false);
    chassis.tank(0, 0, true);    //a turn exits early with the motors still running, so stop and let the heading settle
    pros::delay(150);
    lap("s1");

    //step 2: drive 13.75 in to the goal along -54 (11.12 sideways, 8.08 ahead) at speed 65, then drop hard to speed 29 for the last 4 in, score
    driveSlowEnd(-11.12, 8.08, 4, 29, 65, 500);
    lap("s2drive");
    score(0, -54, 0.2);    //was 0.3, then 1 in less (-0.7), then -0.85, then 0.35 in more back three times (-0.5, -0.15, 0.2)
    lap("s2score");

    //======== PIN 2: the stack next to goal 1, scored on goal 1 at layer 1 ========
    //step 3: raise the claw 30 deg from where it scored, back up 2 in, turn to 65, then lower the claw to horizontal
    endEffector.pivotTo(config::endeffector::pivotPreloadDeg + 70);    //up is positive, the scoring angle is pivotPreloadDeg
    backStraight(2, {.forwards = false, .maxSpeed = 103, .minSpeed = 20, .earlyExitRange = 1}, 2000);
    endEffector.closeClaw();    //after the back-out, close the claw fully; it opens to the pickup width below
    lap("s3back");
    elevator.requestHeight(1.36);    //the height for the pin 2 pickup, was 1.36, then 0.2 higher (1.56), then 0.15 lower (1.41), 0.1 lower (1.31), 0.15 higher (1.46), now 0.1 lower
    chassis.tank(0, 0, true);
    pros::delay(150);    //let the robot stop before turning
    chassis.turnToHeading(65, 4000, {.maxSpeed = 73}, false);
    std::printf("step 3 turn ended at heading %.2f (asked for 65)\n", chassis.getPose().theta);
    endEffector.pivotTo(config::endeffector::pivotHorizontalDeg);    //the claw goes down now that it faces the stack
    lap("s3turn");

    //step 4: open the claw, drive 4.9 in to the stack along 65 (4.44 sideways, 2.07 ahead), clamp, turn to the goal
    endEffector.openClaw(3);    //was -2, 5 deg wider
    chassis.moveToPoint(chassis.getPose().x + 4.44, chassis.getPose().y + 2.07, timeoutMs, {.maxSpeed = 50}, false);
    lap("s4drive");
    endEffector.closeClaw();
    elevator.requestHeight(cfg::goalScoreRotations[1]);
    pros::delay(300);    //let the claw finish closing on the Pin before turning (was 450)
    chassis.turnToHeading(-69, timeoutMs, {.maxSpeed = 103}, false);
    lap("s4turn");

    //step 5: drive 8.5 in to the goal along -69 (7.94 sideways, 3.05 ahead), score
    chassis.moveToPoint(chassis.getPose().x - 7.94, chassis.getPose().y + 3.05, 1500, {.maxSpeed = 45}, false);    //1.5 s: it was waiting out the whole 4 s against the goal
    lap("s5drive");
    score(1, -69, 0.7);
    lap("s5score");

    //======== PIN 3: the matchloader, scored on goal 2 at layer 2 ========
    //step 6: raise the elevator to clear the Pin, back up away from the goal (lowering the elevator), turn to 177
    double clearHeight {cfg::goalScoreRotations[1] - cfg::scoreDropRotations + 0.75};
    elevator.requestHeight(clearHeight);
    waitElevatorNear(clearHeight, 0.1);
    intakeWhenClear();
    chassis.moveToPoint(chassis.getPose().x+25, chassis.getPose().y+2, timeoutMs, {.forwards = false, .maxSpeed = 63}, false);
    endEffector.closeClaw();    //after the back-out, close the claw fully; it opens to the pickup width below
    chassis.turnToHeading(175, timeoutMs, {.maxSpeed = 63}, false);

    //step 7: open the claw for the matchloader (50 deg wider than it was)
    endEffector.openClaw(7);
    pros::delay(200);    //let the claw finish opening

    //step 8: drive 9 in into the matchloader, clamp, back up 7 in, turn to -88
    chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y - 9, timeoutMs, {.maxSpeed = 36}, false);
    endEffector.closeClaw(30);
    pros::delay(300);    //let the claw finish closing
    liftWhenClear(cfg::goalScoreRotations[2], 1);    //the elevator starts rising once the Pin is 1 in clear of the matchloader
    backStraight(7, {.forwards = false, .maxSpeed = 114, .minSpeed = 20, .earlyExitRange = 1}, timeoutMs);
    chassis.turnToHeading(-92, timeoutMs, {.maxSpeed = 114}, false);

    //step 9: drive 29 in to the goal (elevator rising) at speed 55, then drop hard to speed 35 for the last 4 in, score
    elevator.requestHeight(cfg::goalScoreRotations[2]);
    lemlib::Pose beforeStep9 {chassis.getPose()};
    driveSlowEnd(-29, 0, 4, 32, 50, 500);
    score(2, -92, 0.15);

    //======== PIN 4: the matchloader again, scored on goal 2 at layer 3 ========
    //step 10: raise the elevator to clear the Pin, back up to where step 9 started plus 1 in (lowering the elevator), turn to 177
    double lift {cfg::goalScoreRotations[2] - cfg::scoreDropRotations + 0.5};
    elevator.requestHeight(lift);
    waitElevatorNear(lift, 0.1);
    intakeWhenClear();
    chassis.moveToPoint(beforeStep9.x + scoreShiftX + 1, beforeStep9.y + scoreShiftY, timeoutMs, {.forwards = false, .maxSpeed = 63}, false);
    endEffector.closeClaw();    //after the back-out, close the claw fully; it opens to the pickup width below
    chassis.turnToHeading(177, timeoutMs, {.maxSpeed = 63}, false);

    //step 11: open the claw 50 deg wider, drive 11.55 in into the matchloader, clamp, back up 7 in
    endEffector.openClaw(7);
    pros::delay(200);
    chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y - 11.55, timeoutMs, {.maxSpeed = 36}, false);
    endEffector.closeClaw(30);
    pros::delay(300);
    liftWhenClear(cfg::goalScoreRotations[3], 1);
    backStraight(8, {.forwards = false, .maxSpeed = 114, .minSpeed = 20, .earlyExitRange = 1}, timeoutMs);

    //step 12: turn to -90, drive 29.5 in to the goal (elevator rising) at speed 55, then drop hard to speed 35 for the last 4 in, score
    elevator.requestHeight(cfg::goalScoreRotations[3]);
    chassis.turnToHeading(-92, timeoutMs, {.maxSpeed = 114}, false);
    lemlib::Pose beforeStep12 {chassis.getPose()};
    driveSlowEnd(-29.5, 0, 4, 32, 50, 500);
    score(3, -92, 0.15);

    //======== PIN 5: the matchloader again, scored on goal 2 at layer 4 ========
    //step 13: raise the elevator to clear the Pin, back up to where step 12 started minus 1.25 in (lowering the elevator), turn to 173.5
    double secondLift {cfg::goalScoreRotations[3] - cfg::scoreDropRotations + 0.5};
    elevator.requestHeight(secondLift);
    waitElevatorNear(secondLift, 0.1);
    intakeWhenClear();
    chassis.moveToPoint(beforeStep12.x + scoreShiftX - 1.25, beforeStep12.y + scoreShiftY, timeoutMs, {.forwards = false, .maxSpeed = 63}, false);
    endEffector.closeClaw();    //after the back-out, close the claw fully; it opens to the pickup width below
    chassis.turnToHeading(175, timeoutMs, {.maxSpeed = 63}, false);    //3.5 deg left of 177

    //step 14: open the claw 50 deg wider (7 deg less than the other matchloader approaches), drive 12 in into the matchloader, clamp, back up 7 in
    endEffector.openClaw(0);
    pros::delay(200);
    chassis.moveToPoint(chassis.getPose().x + 1.36, chassis.getPose().y - 11.92, timeoutMs, {.maxSpeed = 36}, false);    //12 in along 173.5
    endEffector.closeClaw(30);
    pros::delay(300);
    liftWhenClear(cfg::goalScoreRotations[4], 1);
    backStraight(7, {.forwards = false, .maxSpeed = 114, .minSpeed = 20, .earlyExitRange = 1}, timeoutMs);

    //step 15: turn to -90, drive 29.5 in to the goal (elevator rising) at speed 45, then drop hard to speed 35 for the last 4 in, score
    elevator.requestHeight(cfg::goalScoreRotations[4]);
    chassis.turnToHeading(-90, timeoutMs, {.maxSpeed = 114}, false);
    lemlib::Pose beforeStep15 {chassis.getPose()};
    driveSlowEnd(-29.5, 0, 4, 32, 41, 500);
    score(4, -90, 0.15);

    //======== PIN 6: the matchloader again, scored on goal 3 at layer 2 ========
    //step 16: raise the elevator to clear the Pin, back up to where step 15 started plus 2 in (lowering the elevator),
    //turn to 177, drive 11.5 in into the matchloader, clamp, back up 7 in
    double thirdLift {cfg::goalScoreRotations[4] - cfg::scoreDropRotations + 0.5};
    elevator.requestHeight(thirdLift);
    waitElevatorNear(thirdLift, 0.1);
    intakeWhenClear();
    chassis.moveToPoint(beforeStep15.x + scoreShiftX + 2,beforeStep15.y + scoreShiftY, timeoutMs, {.forwards = false, .maxSpeed = 57}, false);
    endEffector.closeClaw();    //after the back-out, close the claw fully; it opens to the pickup width below
    chassis.turnToHeading(177, timeoutMs, {.maxSpeed = 57}, false);
    endEffector.openClaw(7);
    pros::delay(200);
    chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y - 11.5, timeoutMs, {.maxSpeed = 32}, false);
    endEffector.closeClaw(30);
    pros::delay(300);
    liftWhenClear(cfg::goalScoreRotations[2], 1);
    backStraight(7, {.forwards = false, .maxSpeed = 103, .minSpeed = 20, .earlyExitRange = 1}, timeoutMs);

    //step 17: turn to -26, drive 20 in to the goal along -26 (8.77 sideways, 17.98 ahead) at speed 45, then drop hard to speed 29 for the last 4 in, elevator rising, score
    elevator.requestHeight(cfg::goalScoreRotations[2]);
    chassis.turnToHeading(-26, timeoutMs, {.maxSpeed = 103}, false);
    driveSlowEnd(-8.77, 17.98, 4, 29, 45, 500);
    score(2, -26, 0.25);

    //step 18: back up 18 in (lowering the elevator), turn to -180, drive 8.25 in into the matchloader, clamp, back up 8 in
    intakeWhenClear();
    backStraight(18, {.forwards = false, .maxSpeed = 103}, timeoutMs);
    endEffector.closeClaw();    //after the back-out, close the claw fully; it opens to the pickup width below
    chassis.turnToHeading(-180, timeoutMs, {.maxSpeed = 103}, false);
    endEffector.openClaw(7);
    pros::delay(200);
    chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y - 8.25, timeoutMs, {.maxSpeed = 32}, false);
    endEffector.closeClaw(30);
    pros::delay(300);
    liftWhenClear(cfg::goalScoreRotations[3], 1);
    backStraight(8, {.forwards = false, .maxSpeed = 103, .minSpeed = 20, .earlyExitRange = 1}, timeoutMs);

    //step 19: turn to -26 (6 deg left of -20), drive 19 in to the goal along -26 (8.33 sideways, 17.08 ahead) at speed 45, then drop hard to speed 29 for the last 4 in, elevator rising, score at layer 3
    elevator.requestHeight(cfg::goalScoreRotations[3]);
    chassis.turnToHeading(-26, timeoutMs, {.maxSpeed = 103}, false);
    driveSlowEnd(-8.33, 17.08, 4, 29, 45, 500);
    score(3, -26, 0.25);

    //step 20: back out 8 in (lowering the elevator), close the claw, turn left 26 deg to -52, raise the claw 90 deg, drive 55 in forward along -52 (43.34 sideways to the left, 33.86 ahead)
    intakeWhenClear();
    backStraight(8, {.forwards = false, .maxSpeed = 103}, timeoutMs);
    endEffector.closeClaw();
    chassis.turnToHeading(-52, timeoutMs, {.maxSpeed = 103}, false);
    endEffector.pivotTo(config::endeffector::pivotHorizontalDeg + 90);    //the claw swings up 90 deg from horizontal before the drive
    chassis.moveToPoint(chassis.getPose().x - 43.34, chassis.getPose().y + 33.86, timeoutMs, {.maxSpeed = 80}, false);
}
