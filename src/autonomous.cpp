// Discobots 1104A comp code.
// Marco Tan, Neil Sachdeva
// 
// Code for autonomous goes here.

#include "main.h"

//> Forward Declarations of Autos <//
auto red_auto() -> void;
auto blu_auto() -> void;
auto skl_auto() -> void;

using kMath::Inch;
using kMath::Deg;
using kAuto::k_au_kP; 
using kAuto::k_au_kI; 
using kAuto::k_au_kD; 
using kAuto::k_au_I_Windup_Threshold;

#define NORMAL

auto Ball_Sort_Aut(const pros::vision_object_s_t &ball, int mid_pow) -> int
{
  return ( (ball.signature == op_Sorting_Colour) ? -600 : mid_pow );
}

auto Sorting_Routine() -> void
{
    while (true)
    {
        pros::vision_object_s_t ball { sVision.get_by_size(0) };
        kHardware::Pow_Intake_Convy(500, Ball_Sort_Aut(ball, 600), 600);
        pros::delay(10);
    }
}

auto Move_Back() -> void
{
    while (true)
    {
        kHardware::Drive_Velocity(-50, -50);
        pros::delay(500);
        kHardware::Drive_Velocity(50, 50);
        pros::delay(500);
    }
}

#if defined NORMAL
//> Autonomous Function <//
void autonomous()
{
    // Use a switch statement to determine which auto routine to run.
    kHardware::Pow_Intake_Convy(500, 600);
    pros::delay(500);
    kHardware::Drive_Velocity(50, 50);
    pros::delay(500);
    kHardware::Pow_Intake_Convy(500, 600, 600);
    pros::delay(1250);
    pros::Task move_back { Move_Back, "Move back" };
    pros::Task sort_rout { Sorting_Routine, "Sorting Routine" };
    pros::delay(750);
    sort_rout.suspend();        move_back.suspend();
    sort_rout.remove();         move_back.remove();
    kHardware::Pow_Intake_Convy();
    kHardware::Drive_Velocity();
}

#elif defined LAST_RESORT

void autonomous()
{
    kHardware::Pow_Intake_Convy(0, 600);
    pros::delay(500);
    kHardware::Pow_Intake_Convy(0, 600, 600);
    pros::delay(2000);
    kHardware::Pow_Intake_Convy();
}

#endif
