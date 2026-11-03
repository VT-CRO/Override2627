#pragma once
#include "config/elevator.h"
#include <array>
#include <cstdint>

namespace config::drivetrain{
    //--------  drivetrain constants  -------- 
    //driver control acceleration limits, same idea as the elevator: the voltage may only change so fast.
    //only forward/back (translation) is limited, turning (rotation) is not, then the two are mixed into left/right.
    //in millivolts per second (12000 mV = full). rise = speeding up, fall = slowing down (lowering the voltage
    //on a spinning motor brakes it). blended from Low (elevator down) to High (elevator at top). lower = gentler. TODO tune
    //translation (~17% gentler than the first tune)
    constexpr double voltageRiseLowMvPerSec {25000.0};   //elevator down: 0 to 12 V in 0.48 s
    constexpr double voltageFallLowMvPerSec {25000.0};
    constexpr double voltageRiseHighMvPerSec {7500.0};   //elevator up: 0 to 12 V in 1.6 s
    constexpr double voltageFallHighMvPerSec {7500.0};
    //turning: stick scale and slew (fall faster than rise so it stops quickly). TODO tune with a driver
    //smallest stick output (past the deadband) that still overcomes friction; the speed scaling never shrinks it
    constexpr double minOutput {10.0};
    constexpr double turnScale {0.7};
    constexpr double turnRiseMvPerSec {40000.0};
    constexpr double turnFallMvPerSec {60000.0};
    //top speed (fraction of full) with the elevator at the top, fading linearly from 1.0 with the elevator down. TODO tune
    constexpr double elevatorTopSpeedScale {0.75};

    //turning model V = kS + kV * speed + kA * acceleration (mV, deg/s, deg/s^2) at each elevator height (fraction of its
    //travel), straight-line in between, and the lowest one's values below it. measured by the rotation sysID (hold RIGHT
    //alone 1.5 s), paste its printout here. the lowest is the intake height, not 0: all the way down the elevator touches
    //the drive wheels, so a turn there is not a real one (and the sysID never parks it there)
    //measured 2026-11-02 at 0.20 / 0.60 / 1.00: kA time constants cw / ccw 0.137 / 0.121, 0.182 / 0.163, 0.155 / 0.176 s.
    //kA at 0.60 is 18% over the straight line from 0.20 to 1.00 (more than the ~6% cw / ccw spread), so more heights may
    //help. the old down-only kA 2.742 (from the 63% step time) read about 28% high, as expected
    constexpr double turnLowestHeight {config::elevator::intakeRotations / config::elevator::maxMotorRotations};
    constexpr std::array<double, 3> turnModelHeights {turnLowestHeight, (turnLowestHeight + 1.0) / 2, 1.0};
    constexpr std::array<double, 3> turnKs {1732.616, 1717.653, 1671.702};
    constexpr std::array<double, 3> turnKv {15.300, 15.441, 15.394};
    constexpr std::array<double, 3> turnKa {1.973, 2.660, 2.545};

    //turn feedback, worked out from the model at the elevator's height: critically damped PD at natural frequency
    //turnWn (rad/s), P = kA wn^2, D = 2 kA wn - kV. used by TurnController and LemLib's angular PID (moveToPoint steering).
    //wn 20 with the down model gives LemLib's hand-checked P 11.6, D 115, I 0.068 (was 15: P 6.53, D 71; before that
    //9.7: P 2.74, D 40.27). if a turn overshoots, raise turnDScale a little
    constexpr double turnWn {20.0};
    constexpr double turnDScale {1.15};  //15% over critically damped: with 1.0 the 90 deg test still overshot 0.5-5 deg (sensor lag)
    //LemLib angular PID only (moveToPoint steering): TurnController has no integral
    constexpr double turnIntegralTimeS {1.7};  //how fast the integral makes up for friction, slower = less overshoot
    constexpr double turnWindupDeg {6.0};  //integral only counts within this many degrees of the target

    //TurnController profile (util/turn_controller.h). top speed and acceleration as a share of what the model says the
    //motors can do at 12 V, leaving headroom for the feedback. down model: about 620 deg/s and 3050 deg/s^2
    constexpr double turnSpeedFraction {0.9};
    constexpr double turnAccelFraction {0.8};
    //anti-tip acceleration caps (deg/s^2), elevator down and at the top, straight-line in between. TODO tune: the down
    //cap sits just under the motor limit, the top one is a guess at half
    constexpr double turnMaxAccelLow {3000.0};
    constexpr double turnMaxAccelHigh {1500.0};
    //TurnController's feedback wn, at the intake height and at the top, straight-line in between. the feedforward does
    //most of the work, so it can sit below turnWn. low: in the PC sim (20 ms lag) 8 overshoots 0.7-1.3 deg and 20
    //oscillates, 10-15 settle with no overshoot. high: the 2026-11-02 RIGHT-tap logs with the gyro fixed, at 0.74 and
    //12, show the output swinging +-2 V at about 9 Hz through every slow-down (the robot's speed dipping and coming back,
    //then 1 deg of overshoot), and none at 0.20. the gyro passes that fast wobble straight into D. 8 cuts D at the top
    //from 52 to 29 mV per deg/s. PC sim (which has no 9 Hz wobble) at the top, 90 deg overshoot / 180 deg with the
    //elevator rising overshoot: 12 0 / 0.26 deg, 9 0.06 / 0.72, 8 0.13 / 0.91 (with the kS fade below).
    //TODO check with the log at the top: lower High if the 9 Hz swing is still there, raise it if top turns lag
    constexpr double turnFeedbackWnLow {12.0};
    constexpr double turnFeedbackWnHigh {8.0};
    //jerk caps (deg/s^3), elevator down and at the top, straight-line in between: every change of the turn's
    //acceleration (start, top speed, stop) ramps at most this fast, so the DR4B's sideways flex isn't kicked into a sway.
    //lower = smoother but slower. PC sim (test/turn_controller_sim.cpp, tower sway from the 2026-11-02 sysID), 90 deg
    //at intake / top, time to settle and twist left after: 30000/15000 0.55 s 0.27 deg / 0.69 s 3.1; 15000/6000 0.66 s
    //0.07 / 0.87 s 1.4; 10000/4000 0.75 s 0.05 / 0.98 s 0.9; 6000/3000 0.87 s 0.04 / 1.07 s 0.7
    //TODO tune on the robot: lower until the tower stops ringing after a turn, or raise for speed
    constexpr double turnMaxJerkLow {10000.0};
    constexpr double turnMaxJerkHigh {4000.0};
    //how late the heading (and its rate) reaches the turn loop: the pose updates every 10 ms and the rate is a 10 ms
    //difference. the rotation sysID prints it (from the step's delay): 0.022 measured 2026-11-02, the six steps read
    //0.021-0.057 s before taking off the speed estimate's 15 ms
    constexpr double turnSensorLagS {0.022};
    //turns use the IMU's gyro rate (z axis) as their turn rate, times this to make clockwise positive like the heading.
    //+1 from the RIGHT-tap logs on 2026-11-02: with -1 every turn's gyro rate ran opposite the heading. if it is wrong
    //the turn loop says so in the terminal ("flip ... imuRateSign") and falls back to the heading
    constexpr double imuRateSign {1.0};
    //end integral (TurnController): once the reference has stopped and within turnEndIntegralZoneDeg, it adds P error /
    //turnEndIntegralTimeS per second until the robot breaks static friction (P alone can't within kS / P, about 6 deg).
    //zone = that 6 deg plus margin. PC sim, 180 deg turn that stalls 1.06 deg short, settled in / overshoot: 0.05 s
    //1.38 s / 0.30 deg, 0.1 s 1.69 / 0.11, 0.2 s 1.97 / 0.05, 0.4 s 2.95 / 0. TODO tune: shorter finishes sooner, too
    //short hunts back and forth
    constexpr double turnEndIntegralTimeS {0.05};
    constexpr double turnEndIntegralZoneDeg {8.0};
    //kS fades out below this plan speed while the plan slows down, so the plan's end doesn't step the output by kS.
    //the logged turns' last ~0.1 s ran under 20 deg/s. TODO tune: raise if the end still jerks, lower if it stops short
    constexpr double turnKsFadeDegPerS {20.0};
    //settled = within turnExitDeg and slower than turnExitRateDegPerS for turnExitMs (LemLib's was 1 deg for 100 ms)
    constexpr double turnExitDeg {1.0};
    constexpr double turnExitRateDegPerS {5.0};
    constexpr double turnExitMs {100.0};

    //driver heading hold (no turn stick = drive straight): the same critically damped PD from the model, but gentler.
    //wn 7 is about pushUntilStall's hold strength (P 134 mV per deg). TODO tune: raise until it wiggles, then back off
    constexpr double headingHoldWn {7.0};
    constexpr double headingHoldMaxMv {3000.0};  //cap so a big hit can't spin the robot
    constexpr std::uint32_t headingHoldDelayMs {200};  //after the turn stick drops, let the spin coast out first. TODO tune
}
