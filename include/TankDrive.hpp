#pragma once
#include "main.h"

class TankDrive
{
public:
    TankDrive(pros::MotorGroup *left_motors, pros::MotorGroup *right_motors);
    void tank(int left, int right);

private:
    pros::MotorGroup *left_motors;
    pros::MotorGroup *right_motors;
};