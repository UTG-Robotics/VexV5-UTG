#pragma once
#include "main.h"

class TankDrive
{
public:
    TankDrive(pros::MotorGroup *left_motors, pros::MotorGroup *right_motors);
    void tank(int left, int right);
    void forwardPID(double distance, double max_speed);
    void followProfileForward(std::vector<double> profile);
    void setBrake(double brake_factor, double max);

private:
    void setTank(int left, int right);
    double getRightEncoders();
    double getLeftEncoders();
    double getLeftVelocity();
    double getRightVelocity();
    pros::MotorGroup *left_motors;
    pros::MotorGroup *right_motors;
    pros::Imu *imu;
    // pros::Rotation left_rotation;
    // pros::Rotation right_rotation;

    const double TPI = (1.0 / (300.0 * (7.0 / 3.0))) * 4.1 * M_PI;
    double brake_factor = 0;
    double brake_max = 0;
    PID *drive_PID;
    PID *profile_PID;
    PID *brake_PID;
    // PID *heading_PID;
    // PID *turn_PID;
};