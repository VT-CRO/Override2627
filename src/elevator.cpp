#include "elevator.h"
#include "constants.h"
#include "api.h"
#include "lemlib/api.hpp"
#include <cmath>

namespace elevator{

    pros::MotorGroup elevatorMotors ({constants::rightElevator, constants::leftElevator}, constants::elevatorGearRatio);

    //Homing: timed home, only used on startup
    //Returning: driving to encoder zero with position control (B tap)
    //ManualHoming: driving down for as long as the driver holds B, zeroes on release
    //Manual: driving up/down for as long as the driver holds an arrow (after a 300 ms hold)
    //ManualHold: arrow released, holding wherever manual stopped
    enum class Mode { Homing, Stowed, Positioning, Returning, ManualHoming, Manual, ManualHold };

    static Mode currentMode {Mode::Homing};
    static int level {0};
    static bool homed {false};

    static uint32_t homingStartTime {0};
    static int returnStartTime {0};
    //when B was pressed, 0 when B is not held. used to tell a tap from a hold
    static int bPressStartTime {0};
    //same idea for Up/Down: when an arrow was pressed, 0 when neither is held
    static int arrowPressStartTime {0};
    //power used by Manual, set by handleInput (positive = up)
    static int manualPower {0};

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
            return constants::elevatorMajorIncrements[lastAnchor] * constants::maxMotorRotations;
        }

        double start {constants::elevatorMajorIncrements[section]};
        double end {constants::elevatorMajorIncrements[section + 1]};

        //cast to double so 1/3 and 2/3 don't round down to 0
        double fraction {start + (static_cast<double>(step) / constants::elevatorStepsPerSection) * (end - start)};

        return fraction * constants::maxMotorRotations;
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
        switch(currentMode){
            //braces are needed around a case that declares variables
            case Mode::Homing: {
                //start the clock on the first update after homing was requested
                if(homingStartTime == 0){
                    homingStartTime = pros::millis();
                }

                uint32_t elapsed {pros::millis() - homingStartTime};

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
                elevatorMotors.move(0);
                break;

            case Mode::Positioning:
                //motor's built in position PID holds the current level
                elevatorMotors.move_absolute(positionForLevel(level), constants::elevatorMoveVelocity);
                break;

            case Mode::Returning: {
                //start the clock on the first update after the return was requested
                if(returnStartTime == 0){
                    returnStartTime = pros::millis();
                }

                //come down quickly under position control
                elevatorMotors.move_absolute(0, constants::elevatorMoveVelocity);

                bool atZero {std::abs(elevatorMotors.get_position()) < constants::elevatorZeroTolerance};
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
                elevatorMotors.move(constants::elevatorHomingVolts);
                break;

            case Mode::Manual: {
                //drive with the arrow, but stop at the top and at zero (B hold is for going below zero)
                double position {elevatorMotors.get_position()};
                bool atTop {position >= constants::maxMotorRotations && manualPower > 0};
                bool atBottom {position <= 0 && manualPower < 0};

                if(atTop || atBottom){
                    elevatorMotors.move(0);
                }
                else{
                    elevatorMotors.move(manualPower);
                }
                break;
            }

            case Mode::ManualHold:
                //brake mode is hold, so move(0) keeps the elevator where manual stopped
                elevatorMotors.move(0);
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
        bool upHeld {constants::master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) != 0};
        bool downHeld {constants::master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) != 0};

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

        //track how long an arrow has been held to tell a step from manual
        if(upPressed || downPressed){
            arrowPressStartTime = pros::millis();
        }
        if(!upHeld && !downHeld){
            arrowPressStartTime = 0;
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

        //arrow held long enough: drive manually. the tap already stepped a level, manual takes over from there
        if(arrowPressStartTime != 0 && pros::millis() - arrowPressStartTime > constants::elevatorHoldToHomeMs){
            manualPower = upHeld ? constants::elevatorManualPower : -constants::elevatorManualPower;
            currentMode = Mode::Manual;
            return;
        }

        //arrow released after manual: hold here, and snap level so the next Up/Down tap steps from here
        if(currentMode == Mode::Manual){
            level = nearestLevel(elevatorMotors.get_position());
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
}