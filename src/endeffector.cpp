#include "constants.h"
#include "api.h"
#include "endeffector.h"
#include "elevator.h"
#include <cmath>
#include <algorithm>

namespace endeffector{
    pros::Motor endEffector(constants::endEffectorMotor, constants::endEffectorGears);

    //one motor does both the claw and the wheels: Intake closes the claw and spins the wheels in,
    //Hold keeps a little power on once a piece is in, Opening opens the claw (low power, wheels stay still).
    //Closed (start of match) and Open just brake where the claw is
    enum class Mode{Closed, Intake, Hold, Opening, Open};

    static Mode currentMode {Mode::Closed};
    static uint32_t openStartTime {0};
    //motor position (rotations) when the claw started opening, the open distance is measured from here
    static double openStartPosition {0};
    static uint32_t intakeStartTime {0};
    //when the motor first looked stalled on a piece, 0 while it isn't
    static uint32_t stalledSince {0};

    //voltage we last sent (mV, slew limited) and when we last sent it
    static double appliedVoltage {0};
    static uint32_t lastVoltageTime {0};

    void init(){
        endEffector.set_encoder_units_all(pros::MotorUnits::rotations);
        endEffector.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
        endEffector.brake();
    }

    void update(){
        double targetMv {0};
        switch(currentMode){
            case(Mode::Closed):
            case(Mode::Open):
            break;

            case(Mode::Intake):
            targetMv = constants::endEffectorIntakeMv;
            //a piece stalls the wheels: current high AND speed low. ignore the spin up spike at the start,
            //then require both for a moment before switching to the low hold power and lifting the elevator
            if(pros::millis() - intakeStartTime > constants::endEffectorIntakeGraceMs
               && endEffector.get_current_draw() > constants::endEffectorStallCurrentMa
               && std::abs(endEffector.get_actual_velocity()) < constants::endEffectorStallRpm){
                if(stalledSince == 0){
                    stalledSince = pros::millis();
                }
                if(pros::millis() - stalledSince > constants::endEffectorStallConfirmMs){
                    currentMode = Mode::Hold;
                    elevator::requestLift(constants::elevatorPieceLiftInches);
                }
            }
            else{
                stalledSince = 0;
            }
            break;

            case(Mode::Hold):
            targetMv = constants::endEffectorHoldMv;
            break;

            case(Mode::Opening):
            //opening drives the motor backwards, so the position goes down
            if(openStartPosition - endEffector.get_position() >= constants::endEffectorOpenRotations
               || pros::millis() - openStartTime >= constants::endEffectorOpenTimeoutMs){
                currentMode = Mode::Open;
            }
            else{
                targetMv = -constants::endEffectorOpenMv;
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

    static bool isOpen(){
        return currentMode == Mode::Opening || currentMode == Mode::Open;
    }

    //R1 toggles: open -> intake, anything else (closed, intaking, holding a piece) -> open.
    //the first press of the match opens the claw. intake keeps running until a piece is in (then it drops to
    //low hold power and the elevator lifts) or R1 is pressed again to reopen after a missed pickup.
    //ignored during the startup home
    void handleInputs(){
        if(constants::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) == 0 || elevator::isHoming()){
            return;
        }
        if(isOpen()){
            currentMode = Mode::Intake;
            intakeStartTime = pros::millis();
            stalledSince = 0;
        }
        else{
            requestRelease();
        }
    }

    //open the claw to the set position. does nothing if it is already open, so it never opens past that spot
    void requestRelease(){
        if(isOpen()){
            return;
        }
        currentMode = Mode::Opening;
        openStartTime = pros::millis();
        openStartPosition = endEffector.get_position();
    }

}
