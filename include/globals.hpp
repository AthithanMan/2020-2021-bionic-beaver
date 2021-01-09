#include "main.h"

#ifndef GLOBALS_HPP
#define GLOBALS_HPP

extern std::shared_ptr<okapi::ChassisController> chassis;
extern std::shared_ptr<okapi::AsyncMotionProfileController> profile_controller;

extern okapi::MotorGroup intakes;
extern okapi::Motor convey_top;
extern okapi::Motor convey_bot;
extern okapi::Controller controller;

enum class auto_select
{
    LIVE,
    SKILLS
};

extern auto_select sel_auto;

#endif