#include "motorGroup.hpp"
MotorGroup::MotorGroup(const std::initializer_list<int> _motors)
{
    for (auto motor : _motors)
    {
        this->motors.push_back(std::make_shared<pros::Motor>(motor));
    }
}