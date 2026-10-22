#include "elevator.h"
#include "constants.h"
#include "api.h"
#include "lemlib/api.hpp"
#include <cmath>

namespace commands{

    //R2: lower the elevator to the intake height
    void score(){
        if(constants::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2) == 1){
            elevator::requestIntake();
        }
    }

}
