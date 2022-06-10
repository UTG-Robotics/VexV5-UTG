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