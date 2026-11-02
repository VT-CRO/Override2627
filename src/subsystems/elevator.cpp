#include "subsystems/elevator.h"
#include "config/elevator.h"
#include "config/ports.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace cfg = config::elevator;

namespace {
constexpr int lastAnchor {static_cast<int>(cfg::majorIncrements.size()) - 1};
//highest level index (12 with 5 major increments and 3 steps each)
constexpr int topLevel {lastAnchor * cfg::stepsPerSection};

//converts a level (0 to top level) into a motor position in rotations
//levels are grouped into sections, one per gap between major increments, each split into stepsPerSection steps
double positionForLevel(int level){
    //keep level in bounds so we never read past the end of the major increments array
    level = std::clamp(level, 0, topLevel);

    //which gap we are in, and how many steps into that gap
    int section {level / cfg::stepsPerSection};
    int step {level % cfg::stepsPerSection};

    //top level sits exactly on the last major increment, nothing to interpolate
    if(section == lastAnchor){
        return cfg::majorIncrements[lastAnchor] * cfg::maxMotorRotations;
    }

    double start {cfg::majorIncrements[section]};
    double end {cfg::majorIncrements[section + 1]};

    //cast to double so 1/3 and 2/3 don't round down to 0
    double fraction {start + (static_cast<double>(step) / cfg::stepsPerSection) * (end - start)};

    return fraction * cfg::maxMotorRotations;
}

//voltage that holds the lift against gravity at a motor position: blended between the two measured heights, and
//fading to 0 through the slack at the bottom where the motor isn't loaded yet
double gravityHoldMv(double rotations){
    double t {std::clamp((rotations - cfg::gravityLowAtRotations)
                         / (cfg::gravityHighAtRotations - cfg::gravityLowAtRotations), 0.0, 1.0)};
    double hold {cfg::gravityLowMv + t * (cfg::gravityHighMv - cfg::gravityLowMv)};
    double slackSpan {cfg::gravityLowAtRotations - cfg::loadedStartRotations};
    double loaded {slackSpan > 0 ? std::clamp((rotations - cfg::loadedStartRotations) / slackSpan, 0.0, 1.0) : 1.0};
    return hold * loaded;
}

//closest level to a motor position, so Up/Down taps after a manual move continue from where the elevator really is
int nearestLevel(double position){
    int best {0};
    for(int i {1}; i <= topLevel; i++){
        if(std::abs(positionForLevel(i) - position) < std::abs(positionForLevel(best) - position)){
            best = i;
        }
    }
    return best;
}
}

Elevator::Elevator(pros::v5::Controller& master)
    : master(master),
      motors({config::ports::rightElevator, config::ports::leftElevator}, config::ports::elevatorGearRatio) {}

//send a voltage to both motors, but only let it change at the slew limits so motion ramps smoothly
//(rise when the voltage is growing or changing direction, fall when it is shrinking toward 0)
void Elevator::applyVoltage(double targetMv){
    uint32_t now {pros::millis()};
    //first call, or we haven't been driving for a while: assume one 20 ms loop
    double dtSec {(lastVoltageTime == 0 || now - lastVoltageTime > 100) ? 0.02 : (now - lastVoltageTime) / 1000.0};
    lastVoltageTime = now;

    bool flipping {(targetMv > 0 && appliedVoltage < 0) || (targetMv < 0 && appliedVoltage > 0)};
    bool growing {std::abs(targetMv) > std::abs(appliedVoltage)};
    double rate {(growing || flipping) ? cfg::voltageRiseMvPerSec : cfg::voltageFallMvPerSec};

    double maxStep {rate * dtSec};
    appliedVoltage += std::clamp(targetMv - appliedVoltage, -maxStep, maxStep);

    motors.move_voltage(static_cast<int>(appliedVoltage));
}

//stop right now and hold (brake mode is hold). each motor locks where it is, so they don't fight
void Elevator::brakeNow(){
    appliedVoltage = 0;
    motors.brake();
}

//closed loop move to target (rotations): P controller -> one voltage -> slew limit -> both motors.
//call every update() while positioning
void Elevator::driveToTarget(double target){
    //new target (ex: another Up tap): start driving again
    if(target != controlTarget){
        controlTarget = target;
        holdSettled = false;
        integral = 0;
        settleSince = 0;
    }

    double error {target - position};

    //already there and braking: leave it alone unless it drifted out of the resettle window.
    //not re-commanding every loop is what keeps it from jittering at the setpoint
    if(holdSettled){
        if(std::abs(error) < cfg::resettleTolerance){
            return;
        }
        holdSettled = false;
        integral = 0;
        settleSince = 0;
    }

    //arrived: in the tolerance AND nearly stopped for the dwell time, then brake and hold
    if(std::abs(error) < cfg::settleTolerance && std::abs(velocity) < cfg::settleSpeed){
        if(settleSince == 0){
            settleSince = pros::millis();
        }
        if(pros::millis() - settleSince >= cfg::settleDwellMs){
            brakeNow();
            holdSettled = true;
            return;
        }
    }
    else{
        settleSince = 0;
    }

    bool inZone {std::abs(error) < cfg::integralZone};
    //gravity feedforward: the load changes with height on a DR4B, so the hold voltage is blended by position
    double output {cfg::kp * error - cfg::kd * velocity + gravityHoldMv(position)};

    if(inZone){
        //I: keeps pushing until the leftover error is gone. clamped (anti-windup) so a stuck elevator can't pile up voltage
        integral += error * 0.02;
        integral = std::clamp(integral, -cfg::integralMaxMv / cfg::ki,
                              cfg::integralMaxMv / cfg::ki);
        output += cfg::ki * integral;
    }
    else{
        integral = 0;
        //far from the target: never so small it can't beat friction
        if(std::abs(output) < cfg::minVoltageMv){
            output = std::copysign(cfg::minVoltageMv, error);
        }
    }

    applyVoltage(std::clamp(output, -cfg::maxVoltageMv, cfg::maxVoltageMv));
}

//true if manual is pushing past the top or below zero (B hold is for going below zero)
bool Elevator::manualPastLimit(double voltage) const {
    bool atTop {position >= cfg::maxMotorRotations && voltage > 0};
    bool atBottom {position <= 0 && voltage < 0};
    return atTop || atBottom;
}

void Elevator::init(){
    motors.set_encoder_units_all(pros::MotorUnits::rotations);
    motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
}

void Elevator::forgetHome(){
    homed = false;
}

//motors only run once the robot is enabled, so this is called from autonomous or the X home, not from init().
//blocks until the home finishes: drives down until it stalls on the bottom stop (or zeroes at its timeout)
void Elevator::homeBlocking(){
    if(homed){
        return;
    }
    mode = Mode::ManualHoming;
    level = 0;
    manualHomeStartTime = 0;
    while(mode == Mode::ManualHoming){
        update();
        pros::delay(20);
    }
}

void Elevator::update(){
    //average both motors' encoders so one slipping motor doesn't skew the position
    std::vector<double> positions {motors.get_position_all()};
    double sum {0};
    for(double p : positions){
        sum += p;
    }
    position = positions.empty() ? 0 : sum / positions.size();

    //speed from the position change, lightly filtered. a long gap since the last update (not running) means no usable reading
    uint32_t now {pros::millis()};
    if(lastPositionTime != 0 && now > lastPositionTime && now - lastPositionTime <= 100){
        double raw {(position - lastPosition) / ((now - lastPositionTime) / 1000.0)};
        velocity += 0.5 * (raw - velocity);
    }
    else{
        velocity = 0;
    }
    lastPosition = position;
    lastPositionTime = now;

    //only Positioning/Returning use the settled flag, anything else means the next move starts fresh
    if(mode != Mode::Positioning && mode != Mode::Intake && mode != Mode::Lift && mode != Mode::Returning){
        holdSettled = false;
        integral = 0;
        settleSince = 0;
    }

    switch(mode){
        //braces are needed around a case that declares variables
        case Mode::Stowed:
            //sitting at the bottom, don't push into the hard stop
            appliedVoltage = 0;
            motors.move(0);
            break;

        case Mode::Positioning:
            //voltage controlled move to the current level, brakes and holds once there
            driveToTarget(positionForLevel(level));
            break;

        case Mode::Intake:
            //the intake height, a bit above the stowed height. brakes and holds once there
            driveToTarget(cfg::intakeRotations);
            break;

        case Mode::Lift:
            driveToTarget(liftTarget);
            break;

        case Mode::Returning: {
            //start the clock on the first update after the return was requested
            if(returnStartTime == 0){
                returnStartTime = pros::millis();
            }

            //come down with the same voltage control as level moves
            driveToTarget(0);

            //holdSettled means driveToTarget got within elevatorSettleTolerance of zero
            bool atZero {holdSettled};
            //safety: if zero can't be reached (encoder drifted below the real bottom), stop trying
            //and unlock. the driver can press X to re-home at the real bottom
            bool timedOut {pros::millis() - returnStartTime > cfg::returnTimeoutMs};

            if(atZero || timedOut){
                mode = Mode::Stowed;
            }
            break;
        }

        case Mode::ManualHoming: {
            //push down until the elevator stalls on the bottom stop, then zero and stop pushing
            if(manualHomeStartTime == 0){
                manualHomeStartTime = pros::millis();
                manualHomeStallSince = 0;
            }
            uint32_t elapsed {pros::millis() - manualHomeStartTime};
            bool manualHomeDone {false};

            appliedVoltage = 0;
            //low fixed voltage: at the stop, torque scales with voltage, so this caps how tight the chain gets
            motors.move_voltage(cfg::homeMv);

            //after the grace time (so the start from rest doesn't look like a stall), "on the stop" = the chain has
            //taken up its slack and current has climbed past the threshold. current is used instead of velocity
            //because the chain keeps creeping as it tightens, so velocity may never drop to ~0
            if(elapsed > cfg::homeGraceMs){
                std::vector<int32_t> currents {motors.get_current_draw_all()};
                int32_t highest {0};
                for(int32_t c : currents){
                    highest = std::max(highest, c);
                }
                double fastest {0};
                for(double v : motors.get_actual_velocity_all()){
                    fastest = std::max(fastest, std::abs(v));
                }
                //on the stop = current at its plateau AND the motors have actually stopped (the chain slows to a
                //crawl first, so current alone would zero while it is still taking up slack)
                if(highest < cfg::homeStallCurrentMa || fastest > cfg::homeStallRpm){
                    manualHomeStallSince = 0;
                }
                else{
                    if(manualHomeStallSince == 0){
                        manualHomeStallSince = pros::millis();
                    }
                    if(pros::millis() - manualHomeStallSince > cfg::homeStallConfirmMs){
                        manualHomeDone = true;
                    }
                }
            }

            //on the bottom stop (this is the real zero), or never got there by the timeout: zero here anyway
            if(manualHomeDone || elapsed > cfg::manualHomeTimeoutMs){
                motors.move(0);
                motors.tare_position_all();
                homed = true;
                level = 0;
                mode = Mode::Intake;
            }
            break;
        }

        case Mode::Manual: {
            //drive with L1/L2, ramping the voltage at the slew limits. manualPower is -127 to 127, scale it to millivolts
            double manualVoltage {manualPower / 127.0 * cfg::maxVoltageMv};
            //going down: slow near zero so stopping at the stow position doesn't overshoot below it
            if(manualVoltage < 0){
                manualVoltage *= std::clamp(position / cfg::manualBottomTaperRotations,
                                            cfg::manualBottomMinScale, 1.0);
            }

            //hard stop at the top and at zero, safety beats smoothness here
            if(manualPastLimit(manualVoltage)){
                brakeNow();
            }
            else{
                applyVoltage(manualVoltage);
            }
            break;
        }

        case Mode::ManualHold:
            //L1/L2 released: ramp the voltage down smoothly, then brake and hold once it reaches 0
            if(appliedVoltage == 0 || manualPastLimit(appliedVoltage)){
                brakeNow();
            }
            else{
                applyVoltage(0);
            }
            break;
    }
}

//stow: drive to encoder zero with position control (see Mode::Returning)
//if we were never homed there is no valid zero to drive to, so it does nothing (X homes)
void Elevator::stow(){
    if(!homed){
        return;
    }

    mode = Mode::Returning;
    level = 0;
    returnStartTime = 0;
}

//move to a level. level 0 is the stow position, so going there stows and re-homes instead of holding against the bottom
void Elevator::goToLevel(int newLevel){
    newLevel = std::clamp(newLevel, 0, topLevel);

    if(newLevel == 0){
        stow();
        return;
    }

    //if already moving, the slew limit carries the current voltage over so a new target mid move doesn't jerk
    level = newLevel;
    mode = Mode::Positioning;
}

void Elevator::handleInput(){
    //read every button once per loop so presses can't pile up while inputs are locked
    bool bPressed {master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B) != 0};
    bool yPressed {master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y) != 0};
    bool aPressed {master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) != 0};
    bool upPressed {master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) != 0};
    bool downPressed {master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN) != 0};

    //L1 = manual up, L2 = manual down, while held
    bool manualUpHeld {master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) != 0};
    bool manualDownHeld {master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) != 0};

    //elevator is locked while it is stowing, other buttons are ignored
    if(mode == Mode::Returning){
        return;
    }

    //B tap: go down to zero
    if(bPressed){
        stow();
        return;
    }

    //everything else needs a known zero
    if(!homed){
        return;
    }

    //L1/L2 held: drive manually right away. if both are held, up wins
    if(manualUpHeld || manualDownHeld){
        manualPower = manualUpHeld ? cfg::manualPower : -cfg::manualPower;
        mode = Mode::Manual;
        return;
    }

    //L1/L2 released after manual: hold here, and snap level so the next Up/Down tap steps from here
    if(mode == Mode::Manual){
        level = nearestLevel(position);
        mode = Mode::ManualHold;
        return;
    }

    //Y, A jump to the major heights (major increment 1, 3). X is the home button now (see opcontrol)
    if(yPressed){
        goToLevel(1 * cfg::stepsPerSection);
    }
    else if(aPressed){
        goToLevel(3 * cfg::stepsPerSection);
    }
    //up and down step one level at a time
    else if(upPressed){
        //from stowed, level is 0 so this goes to level 1
        goToLevel(level + 1);
    }
    else if(downPressed && (mode == Mode::Positioning || mode == Mode::ManualHold)){
        //stepping down to level 0 stows and re-homes through goToLevel
        goToLevel(level - 1);
    }
}

//safe to call every loop: does nothing if it is already on its way. level 0 so Up/Down taps step from the bottom
void Elevator::requestIntake(){
    if(!homed){
        return;
    }
    if(mode != Mode::Intake){
        mode = Mode::Intake;
        level = 0;
    }
}

void Elevator::requestHeight(double rotations){
    if(!homed){
        return;
    }
    liftTarget = std::clamp(rotations, 0.0, cfg::maxMotorRotations);
    level = nearestLevel(liftTarget);
    mode = Mode::Lift;
    //the update loop may not have seen the new target yet, so don't report the old one as settled
    holdSettled = false;
}

double Elevator::rotations() const {
    return position;
}
