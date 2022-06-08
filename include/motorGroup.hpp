#include <initializer_list>
#include "pros\motors.hpp"
#include <vector>
#include <memory>
class MotorGroup
{
public:
    MotorGroup(const std::initializer_list<int> _motors);

private:
    std::vector<std::shared_ptr<pros::Motor>> motors;
};