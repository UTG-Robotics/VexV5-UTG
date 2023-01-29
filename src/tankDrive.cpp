#include "main.h"

TankDrive::TankDrive(pros::MotorGroup *left_motors, pros::MotorGroup *right_motors)
{
    this->left_motors = left_motors;
    this->right_motors = right_motors;
    this->left_motors->set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
    this->right_motors->set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);

    // Set brake modes

    // this->left_motors->set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
    // this->right_motors->set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);

    drive_PID = new PID(4, 0, 1);
    profile_PID = new PID(4, 0, 0);
    brake_PID = new PID(1, 0, 0);

    this->left_motors->set_reversed(true);
}

void TankDrive::setBrake(double brake_factor, double max)
{
    this->brake_factor = brake_factor;
    this->brake_max = max;
}
void TankDrive::setTank(int left, int right)
{
    this->left_motors->move(left);
    this->right_motors->move(right);
}
void TankDrive::tank(int left, int right)
{
    if (abs(left) + abs(right) <= 5)
    {
        setTank(-getLeftVelocity() * this->brake_factor, -getRightVelocity() * this->brake_factor);
    }
    else
    {
        setTank(left, right);
    }
}

double TankDrive::getLeftEncoders()
{
    std::vector<double> positions = left_motors->get_positions();
    return (positions[0] + positions[1]) / 2;
}

double TankDrive::getRightEncoders()
{
    std::vector<double> positions = right_motors->get_positions();
    return (positions[0] + positions[1]) / 2;
}

double TankDrive::getLeftVelocity()
{
    std::vector<double> positions = left_motors->get_actual_velocities();
    return (positions[0] + positions[1]) / 2;
}

double TankDrive::getRightVelocity()
{
    std::vector<double> positions = right_motors->get_actual_velocities();
    return (positions[0] + positions[1]) / 2;
}

void TankDrive::forwardPID(double distance, double max_speed)
{
    double left_start = getLeftEncoders();
    double right_start = getRightEncoders();

    double left_target = left_start + distance * (1 / TPI);
    double right_target = right_start + distance * (1 / TPI);
    while (true)
    {
        double out = drive_PID->calculate(left_target - getLeftEncoders());
        printf("out: %f, error: %f\n", out, left_target - getLeftEncoders());
        this->tank(out, out);
        pros::delay(20);
    }
}
void TankDrive::followProfileForward(std::vector<double> profile)
{
    printf("size: %i\n", profile.size());
    double left_start = getLeftEncoders();
    double right_start = getRightEncoders();
    for (int i = 0; i < profile.size(); i++)
    {
        double curVel = (getLeftVelocity() + getRightVelocity()) / 2;
        // double out = profile_PID->calculate(curVel - profile.at(i));
        left_motors->move_velocity(profile.at(i) * 142.718633);
        right_motors->move_velocity(profile.at(i) * 142.718633);
        pros::delay(20);
    }
    left_motors->move_velocity(0);
    right_motors->move_velocity(0);
    // while (true)
    // {
    //     printf("position: %f\n", (getLeftEncoders() + getRightEncoders()) / 2 * TPI);
    //     pros::delay(20);
    // }
}
