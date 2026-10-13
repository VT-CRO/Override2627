#include "constants.h"
#include "api.h"
#include "endeffector.h"
#include "elevator.h"
#include <cmath>
#include <algorithm>

namespace endeffector{
    pros::Motor endEffector(constants::endEffectorMotor, constants::endEffectorGears);

    //one motor does both the claw and the wheels: Intake closes the claw and spins the wheels in,
    //Hold keeps a little power on once a piece is in, Release opens the claw (low power, wheels stay still)
    enum class Mode{Idle, Intake, Hold, Release};

    static Mode currentMode {Mode::Idle};
    static uint32_t releaseStartTime {0};
    static uint32_t intakeStartTime {0};
    //when the current first went over the limit, 0 while it is under
    static uint32_t highCurrentSince {0};

    //voltage we last sent (mV, slew limited) and when we last sent it
    static double appliedVoltage {0};
    static uint32_t lastVoltageTime {0};

    void init(){
        endEffector.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
        endEffector.brake();
    }

    void update(){
        double targetMv {0};
        switch(currentMode){
            case(Mode::Idle):
            break;

            case(Mode::Intake):
            targetMv = constants::endEffectorIntakeMv;
            //a piece stalls the motor, so the current stays high. ignore the spin up spike at the start,
            //then require it to stay high for a moment before switching to the low hold power and lifting the elevator
            if(pros::millis() - intakeStartTime > constants::endEffectorIntakeGraceMs
               && endEffector.get_current_draw() > constants::endEffectorStallCurrentMa){
                if(highCurrentSince == 0){
                    highCurrentSince = pros::millis();
                }
                if(pros::millis() - highCurrentSince > constants::endEffectorStallConfirmMs){
                    currentMode = Mode::Hold;
                    elevator::requestLift(constants::elevatorPieceLiftInches);
                }
            }
            else{
                highCurrentSince = 0;
            }
            break;

            case(Mode::Hold):
            targetMv = constants::endEffectorHoldMv;
            break;

            case(Mode::Release):
            if(pros::millis() - releaseStartTime >= constants::endEffectorReleaseMs){
                currentMode = Mode::Idle;
            }
            else{
                targetMv = -constants::endEffectorReleaseMv;
            }
            break;
        }

        uint32_t now {pros::millis()};
        //first call, or we haven't run for a while: assume one 20 ms loop
        double dtSec {(lastVoltageTime == 0 || now - lastVoltageTime > 100) ? 0.02 : (now - lastVoltageTime) / 1000.0};
        lastVoltageTime = now;

        double maxStep {constants::endEffectorVoltageSlewMvPerSec * dtSec};
        appliedVoltage += std::clamp(targetMv - appliedVoltage, -maxStep, maxStep);

        //hold the claw (and any game piece in it) once the voltage has ramped to 0
        if(targetMv == 0 && appliedVoltage == 0){
            endEffector.brake();
        }
        else{
            endEffector.move_voltage(static_cast<int>(appliedVoltage));
        }
    }

    //R1 held: the claw intakes. it drops to low hold power by itself
    //once a piece is in (and the elevator lifts), and keeps that after R1 is released. R2 (score) releases the piece
    void handleInputs(){
        if(constants::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) != 0){
            currentMode = Mode::Intake;
            intakeStartTime = pros::millis();
            highCurrentSince = 0;
        }
        //released without a piece: stop. with a piece (Hold) it keeps holding
        else if(constants::master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) == 0 && currentMode == Mode::Intake){
            currentMode = Mode::Idle;
        }
    }

    //helpers for commands
    void requestRelease(){
        currentMode = Mode::Release;
        releaseStartTime = pros::millis();
    }

}
