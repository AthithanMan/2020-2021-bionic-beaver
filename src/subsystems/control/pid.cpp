//* Discobots 1104A comp code.
//* Marco Tan, Neil Sachdeva, Dev Patel
//*
//* File Created: 2020-10-03
//* Desc: PID class definitons.

// TODO: Add async ability to the PID controller.

//* Headers
#include "main.h"   // Main header.


//* Local defs.
int old_velocity_l {0};
int old_velocity_r {0};
const int max_change {2};


//* Private definitions

double filter(double curr, double last, double comp)
{
    double filter = curr - last;
    if (std::fabs(filter) < comp) {filter = 0;}
    return filter;
}

/// Calculation function for straight line movements.
void a_PID::calculate_str()
{
    // Explicitly convert m_k_Dt for use in pros::delay() later.
    std::uint32_t uint_m_k_Dt = static_cast<std::uint32_t>(m_k_Dt);

    // Output for left and right sides.
    int output_l, output_r;
    double current;
    double starting {h_obj_sensors->get_rotation()};
    double diff;
    double last {0.0};
    int start_time {static_cast<int>(pros::millis())};

    // Loop as long as the chassis has not reached its target.
    while (std::abs(m_targ_dist - (h_obj_sensors->get_enc(h_Encoder_IDs::AVG_SIDES))) >= m_k_t_uncert)
    {
        // Calculate errors.
        current = h_obj_sensors->get_rotation();
        diff = starting - (starting + filter(current, last, 0.01));
        m_err_l = m_targ_l - h_obj_sensors->get_enc(h_Encoder_IDs::LEFT);
        m_err_r = m_targ_r - h_obj_sensors->get_enc(h_Encoder_IDs::RIGHT);
        last = current;

        // Calculate derivatives.
        m_derv_l = (m_err_l - m_lst_err_l) / m_k_Dt;
        m_derv_r = (m_err_r - m_lst_err_r) / m_k_Dt;

        // Explicitely convert the values to integers to write to the motors.
        output_l = static_cast<int>(std::round((m_err_l * m_kP) + (m_derv_l * m_kD) + (diff * 1.25)));
        output_r = static_cast<int>(std::round((m_err_r * m_kP) + (m_derv_r * m_kD) - (diff * 1.25)));

        // Check whether the values are too high or too low to be used (left side).
        if (std::abs(output_l) > k_Auto::a_max_str_speed)
            output_l = std::copysign(k_Auto::a_max_str_speed, output_l);
        else if (std::abs(output_l) < k_Auto::a_min_str_speed)
            output_l = std::copysign(k_Auto::a_min_str_speed, output_l);
        // Check whether the values are too high or too low to be used (right side).
        if (std::abs(output_r) > k_Auto::a_max_str_speed)
            output_r = std::copysign(k_Auto::a_max_str_speed, output_l);
        else if (std::abs(output_r) < k_Auto::a_min_str_speed)
            output_r = std::copysign(k_Auto::a_min_str_speed, output_r);

        // Slew the values to lower jerk.
        output_l = std::clamp(output_l, old_velocity_l - max_change, old_velocity_l + max_change);
        output_r = std::clamp(output_r, old_velocity_r - max_change, old_velocity_r + max_change);

        // Set previous errors.
        m_lst_err_l = m_err_l;
        m_lst_err_r = m_err_r;

        // Set old velocities.
        old_velocity_l = output_l;
        old_velocity_r = output_r;

        // Set output power to the drive.
        h_obj_chassis->drive_vel(output_l, output_r);

        // Delay because the OCRs cannot record values faster than this.
        pros::lcd::print(2, "%f", diff);
        pros::delay(10);
    }
}


/// Calculation function for point turn movements.
void a_PID::calculate_p_trn()
{
    std::uint32_t uint_m_k_Dt = static_cast<std::uint32_t>(m_k_Dt);

    int output_l, output_r;
    double current_val;
    double filter_val;
    double last_val {0.0};
    
    while (std::fabs(m_targ_head - h_obj_sensors->get_heading()) > m_k_h_uncert)
    {
        current_val = h_obj_sensors->get_heading();
        filter_val += filter(current_val, last_val, 0.1);
        // Calculate the shortest angle towards the target from current heading.
        double diff { fmod( (m_targ_head - filter_val + 180.0), 360.0) - 180 };
        double theta { ( (diff < -180) ? diff + 360 : diff ) };
        last_val = current_val;

        // Set errors.
        m_err_l = theta;
        m_err_r = -theta;

        // Calculate derivatives.
        m_derv_l = (m_err_l - m_lst_err_l) / m_k_Dt;
        m_derv_r = (m_err_r - m_lst_err_r) / m_k_Dt;

        // Explicitely convert the values to integers to write to the motors.
        output_l = static_cast<int>(std::round((m_err_l * m_kP) + (m_derv_l * m_kD)));
        output_r = static_cast<int>(std::round((m_err_r * m_kP) + (m_derv_r * m_kD)));

        // Check whether the values are too high or too low to be used (left side).
        if (std::abs(output_l) > k_Auto::a_max_p_trn_speed)
            output_l = std::copysign(k_Auto::a_max_p_trn_speed, output_l);
        else if (std::abs(output_l) < k_Auto::a_min_p_trn_speed)
            output_l = std::copysign(k_Auto::a_min_p_trn_speed, output_l);
        // Check whether the values are too high or too low to be used (right side).
        if (std::abs(output_r) > k_Auto::a_max_p_trn_speed)
            output_r = std::copysign(k_Auto::a_max_p_trn_speed, output_r);
        else if (std::abs(output_r) < k_Auto::a_min_p_trn_speed)
            output_r = std::copysign(k_Auto::a_min_p_trn_speed, output_r);

        // Slew the values to lower jerk.
        output_l = std::clamp(output_l, old_velocity_l - max_change, old_velocity_l + max_change);
        output_r = std::clamp(output_r, old_velocity_r - max_change, old_velocity_r + max_change);

        // Set previous errors.
        m_lst_err_l = m_err_l;
        m_lst_err_r = m_err_r;

        // Set old velocities.
        old_velocity_l = output_l;
        old_velocity_r = output_r;

        // Set output power to the drive.
        h_obj_chassis->drive_vel(output_l, output_r);

        // Delay because the OCRs cannot record values faster than this.
        pros::delay(10);
    }
}

//* Public definitions

/// class - PID controller. 
/// \param gains An a_PID_Gains struct with all gain values.
a_PID::a_PID(const a_PID_Gains &gains)
    : m_kP{gains.gn_kP}, m_kI{gains.gn_kI}, m_kD{gains.gn_kD}, 
      m_k_Dt{gains.gn_k_Dt}, m_k_min_intg{gains.gn_k_min_intg},
      m_k_t_uncert{gains.gn_k_t_uncert}, m_k_h_uncert{gains.gn_k_h_uncert}
    {}

/// Resets all targets, errors, calculated gains, and sensor readings.
a_PID& a_PID::reset()
{
    m_targ_dist = 0.0;
    m_targ_head = 0.0;
    m_targ_l = 0.0;
    m_targ_r = 0.0;

    m_err_l = 0.0;
    m_err_r = 0.0;
    m_lst_err_l = 0.0;
    m_lst_err_r = 0.0;
    m_derv_l = 0.0;
    m_derv_r = 0.0;

    h_obj_sensors->reset_enc();
    
    return *this;
}

/// Sets the gains of the PID controller.
/// \param gains An a_PID_Gains struct with all gain values.
a_PID& a_PID::set_gains(const a_PID_Gains &gains)
{
    m_kP = gains.gn_kP;
    m_kI = gains.gn_kI;
    m_kD = gains.gn_kD;
    m_k_Dt = gains.gn_k_Dt;
    m_k_min_intg = gains.gn_k_min_intg;
    m_k_t_uncert = gains.gn_k_t_uncert;
    m_k_h_uncert = gains.gn_k_h_uncert;
    return *this;
}

/// Sets the distance target of the PID controller.
/// \param dist_target Target distance in ticks.
a_PID& a_PID::set_target(const a_Ticks &dist_target)
{
    m_targ_dist = dist_target.var;
    return *this;
}

/// Sets the heading target of the PID controller.
/// \param head_target Target heading in degrees.
a_PID& a_PID::set_target(const a_Degrees &head_target)
{
    m_targ_head = head_target.var;
    return *this;
}

/// Starts the PID controller with the supplied targets. 
/// Automatically deduces whether it should drive in a straight line, or 
/// do a point turn based on targets supplied.
void a_PID::drive()
{
    // If we detect a distance target...
    if (m_targ_dist)
    {
        // Set the targets.
        m_targ_l = static_cast<int>(m_targ_dist);
        m_targ_r = static_cast<int>(m_targ_dist);

        // Start distance calculation.
        pros::lcd::print(0, "calculating straight");
        calculate_str();
    }
    // Otherwise if it's a heading target...
    else if (m_targ_head)
    {
        if (m_targ_head == 360.0)
            m_targ_head = 0;
        
        // Start heading calculation
        pros::lcd::print(0, "calculating turn");
        calculate_p_trn();
    }

    // Clear old velocities.
    old_velocity_l = 0;
    old_velocity_r = 0;

    // Stop the motors.
    h_obj_chassis->drive_vel();

    // Reset all values.
    reset();
}
