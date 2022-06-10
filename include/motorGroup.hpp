#include <initializer_list>
#include "pros\motors.hpp"
#include <vector>
#include <memory>
class MotorGroup
{
public:
    MotorGroup(const std::initializer_list<int> _motors);
    MotorGroup(const std::initializer_list<int> _motors, const std::initializer_list<bool> _reversed);
    int move(const int voltage);
    int move_absolute(const int voltage);

private:
    std::vector<std::shared_ptr<pros::Motor>> motors;
};