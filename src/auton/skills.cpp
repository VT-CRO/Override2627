#include "auton/auton.h"
#include "config/auton.h"
#include "config/endeffector.h"
#include <cstdio>

namespace cfg = config::auton;

namespace {
constexpr int timeoutMs {cfg::pathTimeoutMs};
//loader row to goal run: from the loader line to where the robot stalls on the goal (these runs are straight, -90)
constexpr double goalRunIn {cfg::loaderLineFromGoalIn - cfg::contactStraightIn};
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
    //where each loader drive ended, in the current goal frame: the loader stalls the robot at the same spot every time, so a
    //different y here means the pose it started the drive from was off
    auto loaderPose {[&](const char* when){
        lemlib::Pose p {chassis.getPose()};
        std::printf("loader %s: pose (%.2f, %.2f) heading %.1f\n", when, p.x, p.y, p.theta);
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
    endEffector.openClaw(20);    //was -2, 5 deg wider
    pros::delay(150);    //let the robot stop before turning
    chassis.turnToHeading(65, 4000, {.maxSpeed = 73}, false);
    std::printf("step 3 turn ended at heading %.2f (asked for 65)\n", chassis.getPose().theta);
    endEffector.pivotTo(config::endeffector::pivotHorizontalDeg);    //the claw goes down now that it faces the stack
    lap("s3turn");

    //step 4: drive 4.9 in to the stack along 65 (4.44 sideways, 2.07 ahead), clamp, turn to the goal
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
    //step 6: raise the wrist to clear the Pin, back up away from the goal (lowering the elevator), turn to 177
    endEffector.pivotTo(config::endeffector::pivotPreloadDeg + 70);    //same wrist raise as after the preload
    intakeWhenClear();
    chassis.moveToPoint(cfg::loaderLineFromGoalIn, 0, timeoutMs, {.forwards = false, .maxSpeed = 63}, false);    //the loader row in the goal frame, same point every trip
    endEffector.closeClaw();    //after the back-out, close the claw fully; it opens to the pickup width below
    chassis.turnToHeading(180, timeoutMs, {.maxSpeed = 63}, false);

    //step 7: wrist back to horizontal, open the claw for the matchloader (50 deg wider than it was)
    endEffector.pivotTo(config::endeffector::pivotHorizontalDeg);
    endEffector.openClaw(-8);
    pros::delay(200);    //let the claw finish opening

    //step 8: drive 9 in into the matchloader, clamp, back up 7 in, turn to -88
    drivetrain.pushUntilStall(36, 2500, 180, cfg::loaderDriveIn);    //stops on the loader (or at the depth cap), no waiting out a timeout
    loaderPose("stalled");
    endEffector.closeClaw(30);
    pros::delay(300);    //let the claw finish closing
    liftWhenClear(cfg::goalScoreRotations[2], 1);    //the elevator starts rising once the Pin is 1 in clear of the matchloader
    backStraight(8, {.forwards = false, .maxSpeed = 114, .minSpeed = 20, .earlyExitRange = 1}, timeoutMs);
    chassis.turnToHeading(-90, timeoutMs, {.maxSpeed = 114}, false);

    //step 9: drive from the loader row to the goal (elevator rising) at speed 50, then drop hard to speed 32 for the last 4 in, score
    elevator.requestHeight(cfg::goalScoreRotations[2]);
    driveSlowEnd(-goalRunIn, 0, 4, 32, 50, 500);
    score(2, -90, 0.15);

    //======== PIN 4: the matchloader again, scored on goal 2 at layer 3 ========
    //step 10: raise the wrist to clear the Pin, back up to the loader row (lowering the elevator), turn to the loader
    endEffector.pivotTo(config::endeffector::pivotPreloadDeg + 70);
    intakeWhenClear();
    chassis.moveToPoint(cfg::loaderLineFromGoalIn, 0, timeoutMs, {.forwards = false, .maxSpeed = 63}, false);
    endEffector.closeClaw();    //after the back-out, close the claw fully; it opens to the pickup width below
    chassis.turnToHeading(180, timeoutMs, {.maxSpeed = 63}, false);

    //step 11: wrist back to horizontal, open the claw 50 deg wider, drive into the matchloader, clamp, back up 8 in
    endEffector.pivotTo(config::endeffector::pivotHorizontalDeg);
    endEffector.openClaw(-8);
    pros::delay(200);
    drivetrain.pushUntilStall(36, 2500, 180, cfg::loaderDriveIn);    //stops on the loader (or at the depth cap), no waiting out a timeout
    loaderPose("stalled");
    endEffector.closeClaw(30);
    pros::delay(300);
    liftWhenClear(cfg::goalScoreRotations[3], 1);
    backStraight(8, {.forwards = false, .maxSpeed = 114, .minSpeed = 20, .earlyExitRange = 1}, timeoutMs);

    //step 12: turn to -90, drive to the goal (elevator rising), then slow for the last 4 in, score
    elevator.requestHeight(cfg::goalScoreRotations[3]);
    chassis.turnToHeading(-90, timeoutMs, {.maxSpeed = 114}, false);
    driveSlowEnd(-goalRunIn, 0, 4, 32, 50, 500);
    score(3, -90, 0.15);

    //======== PIN 5: the matchloader again, scored on goal 2 at layer 4 ========
    //step 13: raise the wrist to clear the Pin, back up to the loader row (lowering the elevator), turn to the loader
    endEffector.pivotTo(config::endeffector::pivotPreloadDeg + 70);
    intakeWhenClear();
    chassis.moveToPoint(cfg::loaderLineFromGoalIn, 0, timeoutMs, {.forwards = false, .maxSpeed = 63}, false);
    endEffector.closeClaw();    //after the back-out, close the claw fully; it opens to the pickup width below
    chassis.turnToHeading(180, timeoutMs, {.maxSpeed = 63}, false);

    //step 14: wrist back to horizontal, open the claw 50 deg wider (7 deg less than the other matchloader approaches), drive into the matchloader, clamp, back up 8 in
    endEffector.pivotTo(config::endeffector::pivotHorizontalDeg);
    endEffector.openClaw(-15);
    pros::delay(200);
    drivetrain.pushUntilStall(36, 2500, 180, cfg::loaderDriveIn);    //stops on the loader (or at the depth cap), no waiting out a timeout
    loaderPose("stalled");
    endEffector.closeClaw(30);
    pros::delay(300);
    liftWhenClear(cfg::goalScoreRotations[4], 1, true);    //5th score: wrist up while the elevator climbs
    backStraight(8, {.forwards = false, .maxSpeed = 114, .minSpeed = 20, .earlyExitRange = 1}, timeoutMs);

    //step 15: turn to -90, drive to the goal (elevator rising), then slow for the last 4 in, score
    elevator.requestHeight(cfg::goalScoreRotations[4]);
    chassis.turnToHeading(-90, timeoutMs, {.maxSpeed = 114}, false);
    driveSlowEnd(-goalRunIn, 0, 4, 32, 50, 500);
    score(4, -90, 0.15);

    //======== PIN 6: the matchloader again, scored on goal 3 at layer 2 ========
    //step 16: raise the wrist to clear the Pin, back up to the loader row (lowering the elevator),
    //turn to the loader, drive into the matchloader, clamp, back up 8 in
    endEffector.pivotTo(config::endeffector::pivotPreloadDeg + 70);
    intakeWhenClear();
    chassis.moveToPoint(cfg::loaderLineFromGoalIn, 0, timeoutMs, {.forwards = false, .maxSpeed = 63}, false);
    endEffector.closeClaw();    //after the back-out, close the claw fully; it opens to the pickup width below
    chassis.turnToHeading(180, timeoutMs, {.maxSpeed = 63}, false);
    endEffector.pivotTo(config::endeffector::pivotHorizontalDeg);
    endEffector.openClaw(-8);
    pros::delay(200);
    drivetrain.pushUntilStall(36, 2500, 180, cfg::loaderDriveIn);    //stops on the loader (or at the depth cap), no waiting out a timeout
    loaderPose("stalled");
    endEffector.closeClaw(30);
    pros::delay(300);
    liftWhenClear(cfg::goalScoreRotations[2], 1);
    backStraight(8, {.forwards = false, .maxSpeed = 114, .minSpeed = 20, .earlyExitRange = 1}, timeoutMs);

    //step 17: turn to face Red B's center and drive to it from wherever the loader trip ended (the aim and the run come from the pose), elevator rising, score
    elevator.requestHeight(cfg::goalScoreRotations[2]);
    driveToGoal(cfg::redBFromRedAX, cfg::redBFromRedAY);
    score(2, -32, 0.25);

    //steps 18-19 and 20-21: Pins 7 and 8 from the loader, scored on Red B at layers 3 and 4. each trip runs in the frame
    //score() just set on Red B (goal at (0, 0)), so both are the same motion
    for(int layer : {3, 4}){
        //back up (lowering the elevator) to the point 23.11 in off the wall level with the loader row, turn to the
        //loader, drive into the matchloader, clamp, back up 8 in. a straight 18 in back-out along the scoring heading ended
        //about 6.6 in from the bottom wall and the turn hit it, this ends 11.44 in from it
        intakeWhenClear();
        chassis.moveToPoint(cfg::redBLoaderRowX, cfg::redBLoaderBackY, timeoutMs, {.forwards = false, .maxSpeed = 103}, false);
        endEffector.closeClaw();    //after the back-out, close the claw fully; it opens to the pickup width below
        chassis.turnToHeading(180, timeoutMs, {.maxSpeed = 103}, false);
        endEffector.openClaw(-8);
        pros::delay(200);
        loaderPose("start");
        //drives until the robot stalls on the loader instead of a fixed distance: this trip starts from the Red B frame,
        //whose position error is bigger, and a fixed distance fell short of the loader
        drivetrain.pushUntilStall(36, 2500, 180);
        loaderPose("stalled");
        endEffector.closeClaw(30);
        pros::delay(300);
        liftWhenClear(cfg::goalScoreRotations[layer], 1, layer == 4);    //5th score: wrist up while the elevator climbs
        backStraight(8, {.forwards = false, .maxSpeed = 103, .minSpeed = 20, .earlyExitRange = 1}, timeoutMs);

        //turn to face Red B's center and drive to it, elevator rising, score
        elevator.requestHeight(cfg::goalScoreRotations[layer]);
        driveToGoal(0, 0);
        score(layer, -32, 0.25);
    }

    //step 22: back out 8 in (lowering the elevator), close the claw, turn left 26 deg to -52, raise the claw 90 deg, drive 55 in forward along -52 (43.34 sideways to the left, 33.86 ahead)
    // intakeWhenClear();
    // backStraight(8, {.forwards = false, .maxSpeed = 103}, timeoutMs);
    // endEffector.closeClaw();
    // chassis.turnToHeading(-52, timeoutMs, {.maxSpeed = 103}, false);
    // endEffector.pivotTo(config::endeffector::pivotHorizontalDeg + 90);    //the claw swings up 90 deg from horizontal before the drive
    // chassis.moveToPoint(chassis.getPose().x - 43.34, chassis.getPose().y + 33.86, timeoutMs, {.maxSpeed = 80}, false);
    
}
