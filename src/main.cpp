#include "main.h"
#include "drivetrain.h"
#include "constants.h"
#include "elevator.h"
#include "endeffector.h"
#include "commands.h"
#include "auton.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
//homes the elevator and the claw at the same time; returns once both are done. preload: a Pin starts in the claw
static void homeAll(bool preload) {
	pros::Task clawHome([preload] { endeffector::homeBlocking(preload); });
	elevator::homeBlocking();
	clawHome.join();
}

void initialize() {
	drivetrain::init();
	elevator::init();
	endeffector::init();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	auton::select();
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	//the robot starts autonomous holding its preload Pin, so this is the preload home
	homeAll(true);
	auton::run();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	//nothing homes on its own here: X homes (an empty claw), and a real autonomous or the X + Y test homes with the preload
	//true while X is held together with Y, so letting go of X + Y doesn't also count as an X tap
	bool xWithY {false};

	while(true){
		bool xHeld {constants::master.get_digital(pros::E_CONTROLLER_DIGITAL_X) != 0};
		bool yHeld {constants::master.get_digital(pros::E_CONTROLLER_DIGITAL_Y) != 0};
		xWithY = xWithY || (xHeld && yHeld);

		//X + Y runs the selected auton (B or a stick stops it). the run drives everything, the elevator included
		if(auton::driverTest()){
			pros::delay(20);
			continue;
		}

		//X tap (on release, so X + Y can still start the test): home everything again from scratch, like a fresh start.
		//blocks driving until it's done
		if(constants::master.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_X) != 0){
			if(!xWithY){
				elevator::forgetHome();
				endeffector::forgetHome();
				homeAll(false);
			}
			xWithY = false;
		}

		//drivetrain commands
		drivetrain::drive();

		//hold LEFT + RIGHT for 1.5 s to run the elevator auto-tune (on blocks!)
		elevator::autoTune();

		//elevator control
		elevator::handleInput();
		elevator::update();

		//end effector control
		//commands::score() is off while testing: it shares R2 with the claw test
		endeffector::test();		//20ms periodic delay
		pros::delay(20);
	}
}