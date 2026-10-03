#include "elevator.h"
#include "endeffector.h"
#include "constants.h"
#include "api.h"
#include "lemlib/api.hpp"
#include <cmath>

namespace commands{

    enum class scoreState{Idle, WaitingToDrop, WaitingForElevator};
    static scoreState currentScoreState {scoreState::Idle};
    //when R2 started the score, used for the prong drop delay
    static std::uint32_t scoreStartTime {0};

    void score(){
        bool isR2Pressed {constants::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2) == 1};

        switch(currentScoreState){
            case(scoreState::Idle):
            if(isR2Pressed){
                elevator::requestStow();
                scoreStartTime = pros::millis();
                currentScoreState = scoreState::WaitingToDrop;
            }
            break;

            case(scoreState::WaitingToDrop):
            //elevator is already coming down, prongs hold where they are until the delay passes
            if(pros::millis() - scoreStartTime >= constants::scoreDropDelayMs){
                endeffector::requestTop();
                currentScoreState = scoreState::WaitingForElevator;
            }
            break;

            case(scoreState::WaitingForElevator):
            if(elevator::isElevatorStowed()){
                endeffector::requestPickUp();
                currentScoreState = scoreState::Idle;
            }
            break;
        }
    }

}