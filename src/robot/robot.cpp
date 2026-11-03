#include "robot/robot.h"
#include "config/elevator.h"
#include <cmath>

Robot::Robot()
    : master({pros::E_CONTROLLER_MASTER}),
      elevator(master),
      endEffector(master, elevator),
      drivetrain(master, elevator),
      auton(master, drivetrain, elevator, endEffector) {}

void Robot::init(){
    drivetrain.init();
    elevator.init();
    endEffector.init();
}

void Robot::selectAuton(){
    auton.select();
}

void Robot::homeAll(bool preload){
    pros::Task clawHome([this, preload] { endEffector.homeBlocking(preload); });
    elevator.homeBlocking();
    clawHome.join();
}

void Robot::runAuton(){
    //the robot starts autonomous holding its preload Pin, so this is the preload home
    homeAll(true);
    auton.run();
}

bool Robot::autonTest(){
    if(!auton.testing){
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X) != 0 && master.get_digital(pros::E_CONTROLLER_DIGITAL_Y) != 0){
            auton.testing = true;
            //use up the presses so the elevator doesn't jump to the X / Y level once the run ends
            master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X);
            master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y);
            testTask.reset(new pros::Task([this]{
                //always the preload home, like a real autonomous, even if X homed (empty) earlier
                elevator.forgetHome();
                endEffector.forgetHome();
                runAuton();
                auton.testing = false;
            }));
        }
        return auton.testing;
    }

    //there is no competition switch to disable the robot, so B or either stick stops the run
    bool stop {master.get_digital(pros::E_CONTROLLER_DIGITAL_B) != 0
               || std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) > 30
               || std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) > 30};
    if(stop){
        testTask->remove();
        drivetrain.chassis.cancelAllMotions();
        auton.testing = false;
    }
    return auton.testing;
}

void Robot::driverControl(){
    //nothing homes on its own here: X homes (an empty claw), and a real autonomous or the X + Y test homes with the preload
    //true while X is held together with Y, so letting go of X + Y doesn't also count as an X tap
    bool xWithY {false};

    while(true){
        bool xHeld {master.get_digital(pros::E_CONTROLLER_DIGITAL_X) != 0};
        bool yHeld {master.get_digital(pros::E_CONTROLLER_DIGITAL_Y) != 0};
        xWithY = xWithY || (xHeld && yHeld);

        //X + Y runs the selected auton (B or a stick stops it). the run drives everything, the elevator included
        if(autonTest()){
            pros::delay(20);
            continue;
        }

        //X tap (on release, so X + Y can still start the test): home everything again from scratch, like a fresh start.
        //blocks driving until it's done
        if(master.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_X) != 0){
            if(!xWithY){
                elevator.forgetHome();
                endEffector.forgetHome();
                homeAll(false);
            }
            xWithY = false;
        }

        drivetrain.drive();

        //tap RIGHT (under 1.5 s, without LEFT) to turn 90 deg and print the error and overshoot, for tuning angularController.
        //fires on release, ignored when LEFT is down (LEFT + RIGHT is the elevator auto-tune)
        bool right {master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) != 0};
        bool left {master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) != 0};
        if(right && rightPressedAt == 0){
            rightPressedAt = left ? 1 : pros::millis();
        }
        else if(!right && rightPressedAt != 0){
            if(rightPressedAt != 1 && !left && pros::millis() - rightPressedAt < 1500){
                drivetrain.turnBy(90.0);
            }
            rightPressedAt = 0;
        }

        //hold RIGHT alone for 1.5 s to run the drivetrain rotation sysID (on the floor, elevator homed). it moves the
        //elevator to each test height itself, through this
        drivetrain.rotationSysId([this, requested = -1.0](double fraction) mutable {
            if(fraction != requested){
                elevator.requestHeight(fraction * config::elevator::maxMotorRotations);
                requested = fraction;
            }
            elevator.update();
        });

        //hold LEFT + RIGHT for 1.5 s to run the elevator auto-tune (on blocks!)
        elevator.autoTune();

        elevator.handleInput();
        elevator.update();

        //R1/R2 and the right stick drive the claw and pivot (bench test)
        endEffector.test();
        pros::delay(20);
    }
}
