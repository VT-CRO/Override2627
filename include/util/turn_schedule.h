#pragma once
#include "util/turn_controller.h"

//config::drivetrain -> TurnController's inputs, scheduled by elevator height (fraction of travel, 0 = down, 1 = top).
//kept apart from the controller so it stays config-free; Drivetrain and the PC sim both build their inputs here

//the measured model, straight-line between config's turnModelHeights (the lowest one's values below it)
TurnModel turnModelAt(double height);

//top speed and acceleration as shares of what the model allows at 12 V, the acceleration also under the anti-tip cap,
//and the jerk cap, the caps blended from elevator down to top. speedFraction (0-1] slows just this turn
TurnLimits turnLimitsAt(double height, const TurnModel& model, double speedFraction = 1.0);

//TurnController's feedbackWn: lower up high, so the feedback stays well under the DR4B's sway frequency
double turnFeedbackWnAt(double height);

TurnSettings turnSettings();
