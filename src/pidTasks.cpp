#include "main.h"

TankDrive::auto_mode TankDrive::get_mode()
{
    return this->current_mode;
}
void TankDrive::auto_task_func()
{
    printf("starting\n");
    while (true)
    {
        // printf("Position: %f\n", (getRightEncoders() + getLeftEncoders()) / 2);
        // printf("left1: %f, left2: %f, right1: %f, right2: %f\n", left_motors->get_positions()[0], left_motors->get_positions()[1], right_motors->get_positions()[0], right_motors->get_positions()[1]);
        switch (get_mode())
        {
        case DISABLE:
            break;
        case SWING:
            swing_PID_func();
            break;
        case TURN:
            turn_PID_func();
            break;
        case DRIVE:
            drive_PID_func();
            break;
        }
        pros::delay(10);
    }
}

void TankDrive::swing_PID_func()
{
    double output = std::clamp(swing_PID->calculate(imu->get_rotation()), -max_speed, max_speed);
    // printf("error: %f, output: %f\n", swing_PID->error, output);
    if (is_left)
    {
        right_motors->set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
        setTank(output, 0);
    }
    else
    {
        left_motors->set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
        setTank(0, -output);
    }
}
void TankDrive::turn_PID_func()
{
    double heading_output = std::clamp(turn_PID->calculate(imu->get_rotation()), -max_speed, max_speed);
    setTank(heading_output, -heading_output);
}
void TankDrive::drive_PID_func()
{
    bool headingCorrection = true;
    double l_output = std::clamp(left_PID->calculate(getLeftEncoders()), -max_speed, max_speed);
    double r_output = std::clamp(right_PID->calculate(getRightEncoders()), -max_speed, max_speed);
    double heading_output = this->heading_correction ? heading_PID->calculate(imu->get_rotation()) : 0;
    printf("left: %f, right: %f, heading: %f\n", left_PID->error, right_PID->error, heading_PID->error);
    printf("leftOut: %f, rightOut: %f, headingOut: %f\n", l_output, r_output, heading_output);
    setTank(l_output + heading_output, r_output - heading_output);
}