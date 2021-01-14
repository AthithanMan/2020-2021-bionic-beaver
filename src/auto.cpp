//* stinky opcontrol code

//* headers and stuff
#include "globals.hpp"
#include "main.h"

//* functions
void live(void)
{

}

void skills(void)
{

}

/// main callback
void autonmous(void)
{
    profile_controller = okapi::AsyncMotionProfileControllerBuilder()
        .withLimits({1.0, 2.0, 10.0})
        .withOutput(chassis)
        .buildMotionProfileController();
     
    void opcontrol(){
     profile_controller::GeneratePath({0_ft, 0_ft} , {4_ft, 0_ft}, "A");
     profile_controller::GeneratePath({0_ft, 0_ft} , {0_ft, 1_ft}, "B");
     profile_controller::GeneratePath({0_ft, 0_ft} , {0_ft, -2_ft}, "C");
     
     profile_controller::setTarget("A");
     profile_controller::waitUntilSettled;
         
     profile_controller::setTarget("B");
     profile_controller::waitUntilSettled;
         
     profile_controller::setTarget("C");
     profile_controller::waitUntilSettled;
     }
    
    switch (sel_auto)
    {
        case auto_select::LIVE:
            live();
            break;
        case auto_select::SKILLS:
            skills();
            break;
    }
}
