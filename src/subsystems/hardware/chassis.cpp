//* Discobots 1104A comp code.
//* Marco Tan, Neil Sachdeva, Dev Patel
//*
//* File Created: 2020-09-29
//* Desc: Chassis class definitons.


//* Headers
#include "main.h"   // Main header.


//* Helper Functions

/// Gets the absolute average of the motor encoder positions.
double h_Chassis::avg_motor_pos()
{
    return (std::abs(
        (m_LF.get_position() + m_LB.get_position() + 
         m_RF.get_position() + m_RB.get_position()) / 4));
}

//* Definitions

/// class - Chassis. 
//? This constructor has default values for all values besides the port numbers. 
//? You do not have to specify it all.
/// \param ports        Ports numbers to assign.
/// \param cartridge    Cartridge of all motors.
/// \param enc_unit     Encoder units of all motors.
/// \param brake_mode   Brake mode of all motors.
h_Chassis::h_Chassis(
        const h_Drive_Ports &ports, pros::motor_gearset_e cartridge, 
        pros::motor_encoder_units_e enc_unit, pros::motor_brake_mode_e brake_mode
    )
    : m_LF {ports.pt_LF, cartridge, false, enc_unit}, m_LB {ports.pt_LB, cartridge, false, enc_unit},
      m_RF {ports.pt_RF, cartridge, true, enc_unit},  m_RB {ports.pt_RB, cartridge, true, enc_unit}
    {
        m_LF.set_brake_mode(brake_mode);
        m_LB.set_brake_mode(brake_mode);
        m_RF.set_brake_mode(brake_mode);
        m_RB.set_brake_mode(brake_mode);
    }

/// Sets the brake mode of the chassis.
/// \param brake_mode The brake mode to set the chassis as.
h_Chassis& h_Chassis::set_brake_mode(pros::motor_brake_mode_e brake_mode)
{
    m_LF.set_brake_mode(brake_mode);
    m_LB.set_brake_mode(brake_mode);
    m_RF.set_brake_mode(brake_mode);
    m_RB.set_brake_mode(brake_mode);
    return *this;
}

/// Drives the chassis based on velocity. Max velocity based on chassis cartridge 
/// supplied. Supplying no or zeroed parameters stops the motors. 
/// \param velocity Velocity to both sides.
void h_Chassis::drive_vel(int velocity)
{
    m_LF.move_velocity(velocity);
    m_LB.move_velocity(velocity);
    m_RF.move_velocity(velocity);
    m_RB.move_velocity(velocity);    
}

/// Drives the chassis based on velocity. Max velocity based on chassis cartridge 
/// supplied. 
//! Has no default parameters. You must supply velocities. 
/// \param l_velocity Left side velocity.
/// \param r_velocity Right side velocity.
void h_Chassis::drive_vel(int l_velocity, int r_velocity)
{
    m_LF.move_velocity(l_velocity);
    m_LB.move_velocity(l_velocity);
    m_RF.move_velocity(r_velocity);
    m_RB.move_velocity(r_velocity);
}

/// Drives the chassis based on voltage values returned from the joysticks. 
/// Supplying no or zeroed paramters stops the motors.
/// \param l_voltage Left side voltage.
/// \param r_voltage Right side voltage.
void h_Chassis::drive_vol(int l_voltage, int r_voltage)
{
    m_LF.move(l_voltage);
    m_LB.move(l_voltage);
    m_RF.move(r_voltage);
    m_RB.move(r_voltage);
}

/// Halts all further execution until the chassis has reached its target.
void h_Chassis::wait_until_settled()
{
    if (!is_settled)
    {
        while ((m_LF.get_target_position() - avg_motor_pos()) > k_Hardware::h_mot_pos_range)
            { pros::delay(5); }
    }

    is_settled = true;
}