#pragma once
#include "main.h"

class TankDrive
{
public:
    TankDrive(pros::MotorGroup *left_motors, pros::MotorGroup *right_motors, pros::Imu *imu);
    void tank(int left, int right);
    void forwardPID(double distance, double max_speed, bool correct_heading = true);
    void turnPID(double angle, double max_speed);
    void swingPID(double angle, double max_speed, bool is_left);
    void followProfileForward(std::vector<SetPoint *> profile);
    void setBrake(double brake_factor);
    void setMaxSpeed(double max_speed);
    void autoStop();
    void autoWait(double timeout = 99999);
    void autoWaitUntil(double error);
    void startAuto();
    void endAuto();
    // void set_mode(auto_mode mode);

private:
    enum auto_mode
    {
        DISABLE = 0,
        SWING = 1,
        TURN = 2,
        DRIVE = 3
    };
    auto_mode current_mode;
    auto_mode get_mode();
    void set_mode(auto_mode mode);
    void setTank(int left, int right);
    double getRightEncoders();
    double getLeftEncoders();
    double getLeftVelocity();
    double getRightVelocity();

    void auto_task_func();
    void drive_PID_func();
    void turn_PID_func();
    void swing_PID_func();

    pros::Task auto_task;

    pros::MotorGroup *left_motors;
    pros::MotorGroup *right_motors;
    pros::Imu *imu;
    // pros::Rotation left_rotation;
    // pros::Rotation right_rotation;

    int startTime = 0;
    bool autoOver = false;
    bool heading_correction = true;
    double max_speed = 127;
    bool is_left = true;
    const double TPI = (1.0 / (4.25577 * M_PI)) * (300.0 * (7.0 / 3.0));
    double brake_factor = 0;
    double brake_pos_left = 0;
    double brake_pos_right = 0;

    PID *left_PID;
    PID *right_PID;
    PID *heading_PID;
    PID *turn_PID;
    PID *swing_PID;
    PID *profile_PID;
    PID *brake_PID;
};