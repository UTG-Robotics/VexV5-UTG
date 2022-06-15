#include "motorGroup.hpp"

MotorGroup::MotorGroup(const std::initializer_list<int> _motors)
{
    for (int i = 0; i < _motors.size(); i++)
    {
        motors.push_back(std::make_shared<pros::Motor>(_motors.begin()[i]));
    }
}

MotorGroup::MotorGroup(const std::initializer_list<int> _motors, const std::initializer_list<bool> _reversed)
{
    for (int i = 0; i < _motors.size(); i++)
    {
        motors.push_back(std::make_shared<pros::Motor>(_motors.begin()[i]));
        if (_reversed.begin()[i])
        {
            motors.at(i)->set_reversed(true);
        }
    }
}

int MotorGroup::move(const int voltage)
{
    int out = 1;
    for (auto motor : this->motors)
    {
        int errorCode = motor->move(voltage);
        if (errorCode != 1)
        {
            out = errorCode;
        }
    }
    return out;
}

int MotorGroup::move_absolute(double position, int velocity)
{
    int out = 1;
    for (auto motor : this->motors)
    {
        int errorCode = motor->move_absolute(position, velocity);
        if (errorCode != 1)
        {
            out = errorCode;
        }
    }
    return out;
}

int MotorGroup::move_velocity(int velocity)
{
    int out = 1;
    for (auto motor : this->motors)
    {
        int errorCode = motor->move_velocity(position, velocity);
        if (errorCode != 1)
        {
            out = errorCode;
        }
    }
    return out;
}

int MotorGroup::brake(void)
{
    int out = 1;
    for (auto motor : this->motors)
    {
        int errorCode = motor->brake();
        if (errorCode != 1)
        {
            out = errorCode;
        }
    }
    return out;
}

int MotorGroup::move_voltage(int voltage)

    int MotorGroup::move_voltage(int voltage)
{
    int out = 1;
    for (auto motor : this->motors)
    {
        int errorCode = motor->move_voltage(voltage);
        if (errorCode != 1)
        {
            out = errorCode;
        }
    }
    return out;
}

int MotorGroup::modify_profiled_velocity(int voltage)
{
    int out = 1;
    for (auto motor : this->motors)
    {
        int errorCode = motor->modify_profiled_velocity(voltage);
        if (errorCode != 1)
        {
            out = errorCode;
        }
    }
    return out;
}

int MotorGroup::set_brake_mode(pros::motor_brake_mode_e_t mode)
{
    int out = 1;
    for (auto motor : this->motors)
    {
        int errorCode = motor->set_brake_mode(mode);
        if (errorCode != 1)
        {
            out = errorCode;
        }
    }
    return out;
}

int MotorGroup::set_current_limit(int limit)
{
    int out = 1;
    for (auto motor : this->motors)
    {
        int errorCode = motor->set_brake_mode(limit);
        if (errorCode != 1)
        {
            out = errorCode;
        }
    }
    return out;
}

int MotorGroup::set_encoder_units(pros::motor_encoder_units_e_t units)
{
    int out = 1;
    for (auto motor : this->motors)
    {
        int errorCode = motor->set_encoder_units(units);
        if (errorCode != 1)
        {
            out = errorCode;
        }
    }
    return out;
}

int MotorGroup::set_voltage_limit(int limit)
{
    int out = 1;
    for (auto motor : this->motors)
    {
        int errorCode = motor->set_voltage_limit(limit);
        if (errorCode != 1)
        {
            out = errorCode;
        }
    }
    return out;
}

int MotorGroup::set_zero_position(double position)
{
    int out = 1;
    for (auto motor : this->motors)
    {
        int errorCode = motor->set_zero_position(position);
        if (errorCode != 1)
        {
            out = errorCode;
        }
    }
    return out;
}

int MotorGroup::tare_position(void)
{
    int out = 1;
    for (auto motor : this->motors)
    {
        int errorCode = motor->tare_position(void);
        if (errorCode != 1)
        {
            out = errorCode;
        }
    }
    return out;
}

double MotorGroup::get_target_position(void)
{
    return this->motors.at(0)->get_target_position();
}

int MotorGroup::get_target_velocity(void)
{
    return this->motors.at(0)->get_target_velocity();
}

double MotorGroup::get_actual_velocity(void)
{
    return this->motors.at(0)->get_actual_velocity();
}

int MotorGroup::get_current_draw(void)
{
    return this->motors.at(0)->get_current_draw();
}

int MotorGroup::get_direction(void)
{
    return this->motors.at(0)->get_direction();
}

int MotorGroup::get_efficiency(void)
{
    return this->motors.at(0)->get_efficiency();
}

int MotorGroup::get_faults(void)
{
    return this->motors.at(0)->get_faults();
}

int MotorGroup::get_flags(void)
{
    return this->motors.at(0)->get_flags();
}

double MotorGroup::get_position(void)
{
    return this->motors.at(0)->get_position();
}

double MotorGroup::get_power(void)
{
    return this->motors.at(0)->get_power();
}

int MotorGroup::get_raw_position(void)
{
    return this->motors.at(0)->get_raw_position();
}

double MotorGroup::get_temperature(void)
{
    return this->motors.at(0)->get_temperature();
}

double MotorGroup::get_torque(void)
{
    return this->motors.at(0)->get_torque();
}

int MotorGroup::get_voltage(void)
{
    return this->motors.at(0)->get_voltage();
}

int MotorGroup::get_zero_position_flag(void)
{
    return this->motors.at(0)->get_zero_position_flag();
}

int MotorGroup::motor_is_stopped(void)
{
    return this->motors.at(0)->motor_is_stopped();
}

int MotorGroup::is_over_current(void)
{
    return this->motors.at(0)->is_over_current();
}

int MotorGroup::is_over_temp(void)
{
    return this->motors.at(0)->is_over_temp();
}

pros::motor_brake_mode_e_t MotorGroup::get_brake_mode(void)
{
    return this->motors.at(0)->get_brake_mode();
}

int MotorGroup::get_current_limit(void)
{
    return this->motors.at(0)->get_current_limit();
}

pros::motor_encoder_units_e_t MotorGroup::get_encoder_units(void)
{
    return this->motors.at(0)->get_encoder_units();
}

int MotorGroup::get_voltage_limit(void)
{
    return this->motors.at(0)->get_voltage_limit();
}