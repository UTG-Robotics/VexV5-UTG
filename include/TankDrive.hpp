#pragma once
#include "main.h"

class TankDrive
{
public:
    TankDrive(pros::MotorGroup *left_motors, pros::MotorGroup *right_motors);
    void tank(int left, int right);
    void forwardPID(double distance, double maxSpeed);
    void followProfileForward(std::vector<double> profile);

private:
    pros::MotorGroup *left_motors;
    pros::MotorGroup *right_motors;
    // pros::Imu imu;
    pros::Rotation left_rotation;
    pros::Rotation right_rotation;

    PID drivePID;
    PID headingPID;
    PID turnPID;
};