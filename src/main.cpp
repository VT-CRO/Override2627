#include "main.h"
#include "robot/robot.h"

//everything lives in Robot; these are the hooks PROS calls
static Robot robot;

//runs as soon as the program starts, before any competition mode. keep it to a few seconds
void initialize() { robot.init(); }

//runs while the robot is disabled, after autonomous or opcontrol
void disabled() {}

//runs after initialize() and before autonomous on a field: the autonomous selector
void competition_initialize() { robot.selectAuton(); }

void autonomous() { robot.runAuton(); }

void opcontrol() { robot.driverControl(); }
