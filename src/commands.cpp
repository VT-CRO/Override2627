#include "elevator.h"
#include "endeffector.h"
#include "constants.h"
#include "api.h"
#include "lemlib/api.hpp"
#include <cmath>

namespace commands{

    //R2: release the claw right away while the elevator lowers to the score end height. a new R1 press cancels the release.
    //ignored during the startup home
    void score(){
        if(constants::master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2) == 1 && !elevator::isHoming()){
            elevator::requestScoreEnd();
            endeffector::requestRelease();
        }
    }

}
