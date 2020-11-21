//* Discobots 1104A comp code.
//* Marco Tan, Neil Sachdeva, Dev Patel
//*
//* File Created: 2020-09-26
//* Desc: Main autonomous control code.


//* Headers
#include "main.h"   // Main header.


//* Defines for testing purposes.
#define SECTION_THREE

//* Function declarations for the tasks.
void intake_until_torque(void);
void intake_until_two_torque(void);
void intake_intakes_until_watts(void);


//* Local "global" objects.

// PID gains for straight movements.
a_PID_Gains gains_str {
    k_Auto::a_def_kP, k_Auto::a_def_kI, k_Auto::a_def_kD, 
    k_Hardware::h_max_readtime, k_Auto::a_def_integ_windup, 
    k_Auto::a_def_ocr_tick_range, k_Auto::a_def_imu_head_range};

// PID gains for point turns.
a_PID_Gains gains_p_trn {
    k_Auto::a_p_trn_kP, k_Auto::a_p_trn_kI, k_Auto::a_p_trn_kD, 
    k_Hardware::h_max_readtime, k_Auto::a_def_integ_windup, 
    k_Auto::a_def_ocr_tick_range, k_Auto::a_def_imu_head_range
};

pros::Task intake_task_one {intake_until_torque};
pros::Task intake_task_two {intake_until_two_torque};
pros::Task intake_task_three {intake_intakes_until_watts};


//* Macro functions

// assume we're always intaking 2 balls
void intake_until_torque(void)
{
    while (intake_task_one.notify_take(true, TIMEOUT_MAX))
    {
        h_obj_conveyor->set_vel(600);
        h_obj_intake->set_vel(600);

        pros::delay(500);
        do {pros::delay(20);} while (h_obj_conveyor->m_CB.get_torque() < 0.2);
        pros::delay(100);

        h_obj_conveyor->set_vel();
        h_obj_intake->set_vel();
    }
}

void intake_until_two_torque(void)
{
    while (intake_task_two.notify_take(true, TIMEOUT_MAX))
    {
        h_obj_conveyor->set_vel(600);
        h_obj_intake->set_vel(600);

        pros::delay(500);
        do {pros::delay(20);} while (h_obj_conveyor->m_CB.get_torque() < 0.2);
        pros::delay(50);

        h_obj_conveyor->set_vel();
        pros::delay(500);

        do {pros::delay(20);} while ((h_obj_intake->m_IL.get_power() + h_obj_intake->m_IR.get_power()) / 2 < 4.0);
        pros::delay(1000);

        h_obj_intake->set_vel();
    }
}

void intake_intakes_until_watts(void)
{
    while (intake_task_three.notify_take(true, TIMEOUT_MAX))
    {
        h_obj_intake->set_vel(600);

        pros::delay(500);
        do {pros::delay(20);} while ((h_obj_intake->m_IL.get_power() + h_obj_intake->m_IR.get_power()) / 2 < 5.0);
        pros::delay(1250);

        h_obj_intake->set_vel();
    }
}

//* Routine functions.

// Live routine.
void live()
{
    h_obj_sensors->reset_enc();
    h_obj_intake->set_vel(600);
    a_obj_pid->set_gains(gains_str).set_target(a_Ticks{0.8_ft}).drive();
    h_obj_intake->set_vel();
    h_obj_conveyor->set_vel(600);
    pros::delay(525);
    h_obj_conveyor->set_vel();
    a_obj_pid->set_gains(gains_str).set_target(a_Ticks{-4.6_ft}).drive();
    pros::delay(5);
    a_obj_pid->set_gains(gains_p_trn).set_target(a_Degrees{180.0}).drive();
    pros::delay(5);
    a_obj_pid->set_gains(gains_str).set_target(a_Ticks{1.2_ft}).drive();
    h_obj_conveyor->set_vel(-200);
    pros::delay(250);
    h_obj_conveyor->set_vel(600);
    h_obj_intake->set_vel(600);
    pros::delay(900);
    h_obj_conveyor->set_vel();
    h_obj_intake->set_vel();
    a_obj_pid->set_gains(gains_str).set_target(a_Ticks{-1.9_ft}).drive();
    pros::delay(5);
    a_obj_pid->set_gains(gains_p_trn).set_target(a_Degrees{112.5}).drive();
    h_obj_intake->set_vel(600);
    a_obj_pid->set_gains(gains_str).set_target(a_Ticks{5.0_ft}).drive();
    pros::delay(100);
    h_obj_intake->set_vel();
    h_obj_conveyor->set_vel(600);
    pros::delay(1000);
    h_obj_conveyor->set_vel();
    a_obj_pid->set_gains(gains_str).set_target(a_Ticks{-1.0_ft}).drive();
}

// Skills routine.
void skills()
{
    h_obj_sensors->reset_enc();

    h_obj_conveyor->set_vel(600, 0);
    pros::delay(300);
    h_obj_conveyor->set_vel();

    intake_task_two.notify();
    a_obj_pid->set_gains(gains_str).set_target(a_Ticks{5.95_ft}).drive();
    pros::delay(10);
    a_obj_pid->set_gains(gains_p_trn).set_target(a_Degrees{180.0}).drive();
    pros::delay(10);
    a_obj_pid->set_gains(gains_str).set_target(a_Ticks{2.6_ft}).drive();
    h_obj_conveyor->set_vel(600, 600);
    pros::delay(200);
    h_obj_conveyor->set_vel(600, 0);
    pros::delay(700);
    h_obj_conveyor->set_vel();

    a_obj_pid->set_gains(gains_str).set_target(a_Ticks{-2.4_ft}).drive();
    h_obj_conveyor->set_vel(0, 600);
    h_obj_intake->set_vel(600);
    pros::delay(500);
    h_obj_conveyor->set_vel();
    h_obj_intake->set_vel();
    a_obj_pid->set_gains(gains_p_trn).set_target(a_Degrees{128.0}).drive();
    pros::delay(10);
    intake_task_three.notify();
    a_obj_pid->set_gains(gains_str).set_target(a_Ticks{5.2_ft}).drive();
    h_obj_conveyor->set_vel(600, 600);
    pros::delay(200);
    h_obj_conveyor->set_vel(600, 0);
    pros::delay(700);
    h_obj_conveyor->set_vel();
    a_obj_pid->set_gains(gains_str).set_target(a_Ticks{-1.7_ft}).drive();
    h_obj_conveyor->set_vel(0, 600);
    h_obj_intake->set_vel(600);
    pros::delay(500);
    h_obj_conveyor->set_vel();
    h_obj_intake->set_vel();

    a_obj_pid->set_gains(gains_p_trn).set_target(a_Degrees{43.0}).drive();
    pros::delay(10);
    a_obj_pid->set_gains(gains_str).set_target(a_Ticks{3.0_ft}).drive();
    h_obj_conveyor->set_vel(600);
    pros::delay(900);
    h_obj_conveyor->set_vel();
    a_obj_pid->set_gains(gains_str).set_target(a_Ticks{-2.5_ft}).drive();
    pros::delay(10);
    a_obj_pid->set_gains(gains_p_trn).set_target(a_Degrees{25.75}).drive();
    pros::delay(10);
    intake_task_one.notify();
    a_obj_pid->set_gains(gains_str).set_target(a_Ticks{7.35_ft}).drive();
    h_obj_conveyor->set_vel(600);
    pros::delay(1000);
    h_obj_conveyor->set_vel();
    a_obj_pid->set_gains(gains_str).set_target(a_Ticks{-3.3_ft}).drive();
}

void telem_a()
{
    while (true)
    {
        pros::lcd::print(1, "%f", h_obj_sensors->get_rotation());
        pros::delay(10);
    }
}

// Main autonomous control callback.
void autonomous()
{
    // Create new a_PID object into heap memory.
    a_obj_pid = new a_PID{gains_str};

    pros::Task foo {telem_a};

    // Run each auto routine based on selected auto.
    switch (a_routine)
    {
    case a_Autonomous_Routine::LIVE:
        live();
        break;
    case a_Autonomous_Routine::SKILLS:
        skills();
        break;
    }
}
