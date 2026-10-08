#include "elevator.h"
#include "constants.h"
#include "api.h"
#include "lemlib/api.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

namespace elevator{

    pros::MotorGroup elevatorMotors ({constants::rightElevator, constants::leftElevator}, constants::elevatorGearRatio);

    enum class Mode { Homing, Stowed, Positioning, Returning, ManualHoming, Manual, ManualHold };

    double currentElevatorMotorRotations {0};

    //Homing: timed home, only used on startup
    //Returning: driving to encoder zero with position control (B tap)
    //ManualHoming: driving down for as long as the driver holds B, zeroes on release
    //Manual: driving up/down for as long as the driver holds L1 (up) or L2 (down)
    //ManualHold: L1/L2 released, holding wherever manual stopped

    static Mode currentMode {Mode::Homing};
    static int level {0};
    static bool homed {false};

    static uint32_t homingStartTime {0};
    static int returnStartTime {0};
    //when B was pressed, 0 when B is not held. used to tell a tap from a hold
    static int bPressStartTime {0};
    //power used by Manual, set by handleInput (positive = up)
    static int manualPower {0};

    //voltage control state: the voltage we last sent (mV, slew limited), when we last sent it,
    //the target we are driving to, and whether we reached it and are braking
    static double appliedVoltage {0};
    static uint32_t lastVoltageTime {0};
    static double controlTarget {0};
    static bool holdSettled {false};

    //converts a level (0 to top level) into a motor position in rotations
    //levels are grouped into sections, one per gap between major increments, each split into elevatorStepsPerSection steps
    static double positionForLevel(int level){
        constexpr int lastAnchor {static_cast<int>(constants::elevatorMajorIncrements.size()) - 1};
        constexpr int topLevel {lastAnchor * constants::elevatorStepsPerSection};

        //keep level in bounds so we never read past the end of the major increments array
        level = std::clamp(level, 0, topLevel);

        //which gap we are in, and how many steps into that gap
        int section {level / constants::elevatorStepsPerSection};
        int step {level % constants::elevatorStepsPerSection};

        //top level sits exactly on the last major increment, nothing to interpolate
        if(section == lastAnchor){
            return constants::elevatorMajorIncrements[lastAnchor] * constants::elevatorMaxMotorRotations;
        }

        double start {constants::elevatorMajorIncrements[section]};
        double end {constants::elevatorMajorIncrements[section + 1]};

        //cast to double so 1/3 and 2/3 don't round down to 0
        double fraction {start + (static_cast<double>(step) / constants::elevatorStepsPerSection) * (end - start)};

        return fraction * constants::elevatorMaxMotorRotations;
    }

    void manualCommand(){
        if(constants::master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) == 1){
            elevatorMotors.move(127);
        }
        else if(constants::master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) == 1){
            elevatorMotors.move(-127);
        }
        else{
            elevatorMotors.move(0);
        }
    }

    //send a voltage to both motors, but only let it change at the slew limits so motion ramps smoothly
    //(rise when the voltage is growing or changing direction, fall when it is shrinking toward 0)
    static void applyVoltage(double targetMv){
        uint32_t now {pros::millis()};
        //first call, or we haven't been driving for a while: assume one 20 ms loop
        double dtSec {(lastVoltageTime == 0 || now - lastVoltageTime > 100) ? 0.02 : (now - lastVoltageTime) / 1000.0};
        lastVoltageTime = now;

        bool flipping {(targetMv > 0 && appliedVoltage < 0) || (targetMv < 0 && appliedVoltage > 0)};
        bool growing {std::abs(targetMv) > std::abs(appliedVoltage)};
        double rate {(growing || flipping) ? constants::elevatorVoltageRiseMvPerSec : constants::elevatorVoltageFallMvPerSec};

        double maxStep {rate * dtSec};
        appliedVoltage += std::clamp(targetMv - appliedVoltage, -maxStep, maxStep);

        elevatorMotors.move_voltage(static_cast<int>(appliedVoltage));
    }

    //stop right now and hold (brake mode is hold). each motor locks where it is, so they don't fight
    static void brakeNow(){
        appliedVoltage = 0;
        elevatorMotors.brake();
    }

    //closed loop move to target (rotations): P controller -> one voltage -> slew limit -> both motors.
    //call every update() while positioning
    static void driveToTarget(double target){
        //new target (ex: another Up tap): start driving again
        if(target != controlTarget){
            controlTarget = target;
            holdSettled = false;
        }

        double error {target - currentElevatorMotorRotations};

        //already there and braking: leave it alone unless it drifted out of the resettle window.
        //not re-commanding every loop is what keeps it from jittering at the setpoint
        if(holdSettled){
            if(std::abs(error) < constants::elevatorResettleTolerance){
                return;
            }
            holdSettled = false;
        }

        //arrived: brake and hold
        if(std::abs(error) < constants::elevatorSettleTolerance){
            brakeNow();
            holdSettled = true;
            return;
        }

        //P: voltage fades out as we get close, which is the smooth decel into the target
        double output {std::clamp(constants::elevatorKp * error, -constants::elevatorMaxVoltageMv, constants::elevatorMaxVoltageMv)};
        //but never so small it can't beat friction
        if(std::abs(output) < constants::elevatorMinVoltageMv){
            output = std::copysign(constants::elevatorMinVoltageMv, error);
        }

        applyVoltage(output);
    }

    //true if manual is pushing past the top or below zero (B hold is for going below zero)
    static bool manualPastLimit(double voltage){
        bool atTop {currentElevatorMotorRotations >= constants::elevatorMaxMotorRotations && voltage > 0};
        bool atBottom {currentElevatorMotorRotations <= 0 && voltage < 0};
        return atTop || atBottom;
    }

    //request a home. the timer starts on the first update() after this, not here,
    //because init() runs long before opcontrol when plugged into field control
    void startHoming(){
        currentMode = Mode::Homing;
        level = 0;
        homingStartTime = 0;
    }

    void init(){
        elevatorMotors.set_encoder_units_all(pros::MotorUnits::rotations);
        elevatorMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
        startHoming();
    }

    void update(){
        //average both motors' encoders so one slipping motor doesn't skew the position
        std::vector<double> positions {elevatorMotors.get_position_all()};
        double sum {0};
        for(double p : positions){
            sum += p;
        }
        currentElevatorMotorRotations = positions.empty() ? 0 : sum / positions.size();

        //only Positioning/Returning use the settled flag, anything else means the next move starts fresh
        if(currentMode != Mode::Positioning && currentMode != Mode::Returning){
            holdSettled = false;
        }

        switch(currentMode){
            //braces are needed around a case that declares variables
            case Mode::Homing: {
                //start the clock on the first update after homing was requested
                if(homingStartTime == 0){
                    homingStartTime = pros::millis();
                }

                uint32_t elapsed {pros::millis() - homingStartTime};

                appliedVoltage = 0;

                if(elapsed < constants::elevatorHomingTimeMs){
                    //still driving down. move_voltage takes millivolts, so convert from volts
                    elevatorMotors.move(constants::elevatorHomingVolts);
                }
                else{
                    //driven down long enough to be sitting on the bottom, zero here
                    elevatorMotors.move(0);
                    elevatorMotors.tare_position_all();
                    homed = true;
                    level = 0;
                    currentMode = Mode::Stowed;
                }
                break;
            }

            case Mode::Stowed:
                //sitting at the bottom, don't push into the hard stop
                appliedVoltage = 0;
                elevatorMotors.move(0);
                break;

            case Mode::Positioning:
                //voltage controlled move to the current level, brakes and holds once there
                driveToTarget(positionForLevel(level));
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
                //and unlock. the driver can hold B to re-home at the real bottom
                bool timedOut {pros::millis() - returnStartTime > constants::elevatorReturnTimeoutMs};

                if(atZero || timedOut){
                    currentMode = Mode::Stowed;
                }
                break;
            }

            case Mode::ManualHoming:
                //push down while B is held. handleInput zeroes and stows when B is released
                appliedVoltage = 0;
                elevatorMotors.move(constants::elevatorHomingVolts);
                break;

            case Mode::Manual: {
                //drive with L1/L2, ramping the voltage at the slew limits. manualPower is -127 to 127, scale it to millivolts
                double manualVoltage {manualPower / 127.0 * constants::elevatorMaxVoltageMv};

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

    //highest level index (12 with 5 major increments and 3 steps each)
    static constexpr int topLevel {(static_cast<int>(constants::elevatorMajorIncrements.size()) - 1) * constants::elevatorStepsPerSection};

    //closest level to a motor position, so Up/Down taps after a manual move continue from where the elevator really is
    static int nearestLevel(double position){
        int best {0};
        for(int i {1}; i <= topLevel; i++){
            if(std::abs(positionForLevel(i) - position) < std::abs(positionForLevel(best) - position)){
                best = i;
            }
        }
        return best;
    }

    //stow: drive to encoder zero with position control (see Mode::Returning)
    //if we were never homed there is no valid zero to drive to, so do the timed home instead
    static void stow(){
        if(!homed){
            startHoming();
            return;
        }

        currentMode = Mode::Returning;
        level = 0;
        returnStartTime = 0;
    }

    //move to a level. level 0 is the stow position, so going there stows and re-homes instead of holding against the bottom
    static void goToLevel(int newLevel){
        newLevel = std::clamp(newLevel, 0, topLevel);

        if(newLevel == 0){
            stow();
            return;
        }

        //if already moving, the slew limit carries the current voltage over so a new target mid move doesn't jerk
        level = newLevel;
        currentMode = Mode::Positioning;
    }

    void handleInput(){
        //read every button once per loop so presses can't pile up while inputs are locked
        bool bPressed {constants::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B) != 0};
        bool yPressed {constants::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y) != 0};
        bool xPressed {constants::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X) != 0};
        bool aPressed {constants::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) != 0};
        bool upPressed {constants::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) != 0};
        bool downPressed {constants::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN) != 0};

        bool bHeld {constants::master.get_digital(pros::E_CONTROLLER_DIGITAL_B) != 0};
        //L1 = manual up, L2 = manual down, while held
        bool manualUpHeld {constants::master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) != 0};
        bool manualDownHeld {constants::master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) != 0};

        //manual home: keep pushing down until B is let go, then that spot is the new zero
        if(currentMode == Mode::ManualHoming){
            if(!bHeld){
                elevatorMotors.move(0);
                elevatorMotors.tare_position_all();
                homed = true;
                level = 0;
                bPressStartTime = 0;
                currentMode = Mode::Stowed;
            }
            return;
        }

        //track how long B has been held to tell a tap from a hold
        if(bPressed){
            bPressStartTime = pros::millis();
        }
        if(!bHeld){
            bPressStartTime = 0;
        }

        //B held long enough: switch to manual homing, this overrides whatever the elevator was doing
        if(bPressStartTime != 0 && pros::millis() - bPressStartTime > constants::elevatorHoldToHomeMs){
            currentMode = Mode::ManualHoming;
            level = 0;
            return;
        }

        //elevator is locked while it is stowing or doing the startup home, other buttons are ignored
        if(currentMode == Mode::Returning || currentMode == Mode::Homing){
            return;
        }

        //B tap: go down to zero. if B stays held this turns into a manual home above
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
            manualPower = manualUpHeld ? constants::elevatorManualPower : -constants::elevatorManualPower;
            currentMode = Mode::Manual;
            return;
        }

        //L1/L2 released after manual: hold here, and snap level so the next Up/Down tap steps from here
        if(currentMode == Mode::Manual){
            level = nearestLevel(currentElevatorMotorRotations);
            currentMode = Mode::ManualHold;
            return;
        }

        //Y, X, A jump to the major heights (major increment 1, 2, 3)
        if(yPressed){
            goToLevel(1 * constants::elevatorStepsPerSection);
        }
        else if(xPressed){
            goToLevel(2 * constants::elevatorStepsPerSection);
        }
        else if(aPressed){
            goToLevel(3 * constants::elevatorStepsPerSection);
        }
        //up and down step one level at a time
        else if(upPressed){
            //from stowed, level is 0 so this goes to level 1
            goToLevel(level + 1);
        }
        else if(downPressed && (currentMode == Mode::Positioning || currentMode == Mode::ManualHold)){
            //stepping down to level 0 stows and re-homes through goToLevel
            goToLevel(level - 1);
        }
    }

    //helpers for commands
    void requestStow(){
        stow();
    }

    bool isElevatorStowed(){
        if(!(currentMode == Mode::Stowed)){
            return false;
        }
        return true;
    }

    double getElevatorRotations(){
        return currentElevatorMotorRotations;
    }
}