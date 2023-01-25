#include "main.h"

TankDrive::TankDrive(pros::MotorGroup *left_motors, pros::MotorGroup *right_motors)
{
    this->left_motors = left_motors;
    this->right_motors = right_motors;

    // Set brake modes
    this->left_motors->set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
    this->right_motors->set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);

    this->left_motors->set_reversed(true);
}
void TankDrive::tank(int left, int right)
{
    this->left_motors->move(left);
    this->right_motors->move(right);
}

void TankDrive::forwardPID(double distance, double maxSpeed)
{
}
