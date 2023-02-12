#include "main.h"

TankDrive::TankDrive(pros::MotorGroup *left_motors, pros::MotorGroup *right_motors, pros::Imu *imu) : auto_task([this]
                                                                                                                { this->auto_task_func(); })
{
    this->left_motors = left_motors;
    this->right_motors = right_motors;
    this->left_motors->set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
    this->right_motors->set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
    this->imu = imu;
    // Set brake modes

    // this->left_motors->set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
    // this->right_motors->set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);

    left_PID = new PID(0.25, 0.008, 0.2, 3 * TPI);
    right_PID = new PID(0.25, 0.008, 0.2, 3 * TPI);
    heading_PID = new PID(10, 0.1, 0, 1);

    turn_PID = new PID(6, 0.15, 0.15, 5);

    swing_PID = new PID(10, 0.15, 0.1, 5);

    profile_PID = new PID(4, 0, 0);
    brake_PID = new PID(1, 0, 0);
    this->left_motors->set_reversed(true);
}

void TankDrive::setBrake(double brake_factor)
{
    this->brake_factor = brake_factor;
}
void TankDrive::setTank(int left, int right)
{
    this->left_motors->move(left);
    this->right_motors->move(right);
}
void TankDrive::tank(int left, int right)
{

    double curve_strength = 10;
    if (false)
    {
        left = (powf(2.718, -(curve_strength / 10)) + powf(2.718, (fabs(left) - 127) / 10) * (1 - powf(2.718, -(curve_strength / 10)))) * left;
        right = (powf(2.718, -(curve_strength / 10)) + powf(2.718, (fabs(right) - 127) / 10) * (1 - powf(2.718, -(curve_strength / 10)))) * right;
    }
    if (abs(left) < 5)
    {
        // left = -(brake_pos_left - getLeftEncoders()) * this->brake_factor;
    }
    else
    {
        brake_pos_left = getLeftEncoders();
    }
    if (abs(right) < 5)
    {
        // right = -(brake_pos_right - getRightEncoders()) * this->brake_factor;
    }
    else
    {
        brake_pos_right = getRightEncoders();
    }
    setTank(left, right);
}
// else
// {
//     setTank(-(brake_pos_left - getLeftEncoders()) * this->brake_factor, -(brake_pos_right - getRightEncoders()) * this->brake_factor);
// }
// }

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

void TankDrive::forwardPID(double distance, double max_speed, bool correct_heading)
{
    heading_correction = correct_heading;
    this->max_speed = max_speed;
    left_PID->set_target(getLeftEncoders() + distance * TPI);
    right_PID->set_target(getRightEncoders() + distance * TPI);
    // printf("left: %f, right: %f\n", getLeftEncoders() + distance * TPI, getRightEncoders() + distance * TPI);
    heading_PID->set_target(imu->get_rotation());
    set_mode(TankDrive::auto_mode::DRIVE);
}

void TankDrive::turnPID(double angle, double max_speed)
{
    this->max_speed = max_speed;
    turn_PID->set_target(angle);
    set_mode(TankDrive::auto_mode::TURN);
}
void TankDrive::swingPID(double angle, double max_speed, bool is_left)
{
    this->is_left = is_left;
    this->max_speed = max_speed;
    swing_PID->set_target(angle);
    set_mode(TankDrive::auto_mode::SWING);
}

void TankDrive::set_mode(TankDrive::auto_mode mode)
{
    this->current_mode = mode;
}
void TankDrive::followProfileForward(std::vector<SetPoint *> profile)
{
    printf("size: %i\n", profile.size());
    double left_start = getLeftEncoders();
    double right_start = getRightEncoders();
    for (int i = 0; i < profile.size(); i++)
    {
        double curVel = (getLeftVelocity() + getRightVelocity()) / 2;
        // double out = profile_PID->calculate(curVel - profile.at(i));
        left_motors->move_velocity(profile.at(i)->velocity * 142.718633);
        right_motors->move_velocity(profile.at(i)->velocity * 142.718633);
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

void TankDrive::autoStop()
{
    set_mode(TankDrive::auto_mode::DISABLE);
}

void TankDrive::startAuto()
{
    startTime = pros::millis();
}
void TankDrive::endAuto()
{
    set_mode(TankDrive::auto_mode::DISABLE);
    left_motors->move_velocity(0);
    right_motors->move_velocity(0);
}

void TankDrive::setMaxSpeed(double max_speed)
{
    this->max_speed = max_speed;
}

void TankDrive::autoWait(double timeout)
{
    if (pros::millis() - startTime >= 55000 && !autoOver)
    {
        set_mode(TankDrive::auto_mode::DISABLE);
        turnPID(-135, 127 / 2);
        pros::delay(3000);
        expansion.setExtended(true);
        pros::delay(3000);
    }
    if (get_mode() == TankDrive::auto_mode::DRIVE)
    {
        while (!left_PID->exit(1 * TPI, 0.1, timeout) && !right_PID->exit(1 * TPI, 0.1, timeout))
        {
            pros::delay(20);
            if (pros::millis() - startTime >= 55000 && !autoOver)
            {
                set_mode(TankDrive::auto_mode::DISABLE);
                turnPID(-135, 127 / 2);
                pros::delay(3000);
                expansion.setExtended(true);
                pros::delay(3000);
            }
        }
    }
    else if (current_mode == TankDrive::auto_mode::TURN)
    {
        while (!turn_PID->exit(0.5, 0.1, timeout))
        {
            pros::delay(20);
            if (pros::millis() - startTime >= 55000 && !autoOver)
            {
                set_mode(TankDrive::auto_mode::DISABLE);
                turnPID(-135, 127 / 2);
                pros::delay(3000);
                expansion.setExtended(true);
                pros::delay(3000);
            }
        }
    }
    else if (current_mode == TankDrive::auto_mode::SWING)
    {
        while (!swing_PID->exit(0.5, 0.1, timeout))
        {
            pros::delay(20);
            if (pros::millis() - startTime >= 55000 && !autoOver)
            {
                set_mode(TankDrive::auto_mode::DISABLE);
                turnPID(-135, 127 / 2);
                pros::delay(3000);
                expansion.setExtended(true);
                pros::delay(3000);
            }
        }
    }
    set_mode(TankDrive::auto_mode::DISABLE);
}

void TankDrive::autoWaitUntil(double error)
{
    // printf("Mode: %i", current_mode);
    if (current_mode == TankDrive::auto_mode::DRIVE)
    {
        // printf("error: %f, wait: %f\n", abs(left_PID->error), abs(error * TPI));
        while (abs(left_PID->error) > abs(error * TPI) || abs(right_PID->error) > abs(error * TPI))
        {
            // printf("error: %f, wait: %f\n", abs(left_PID->error), abs(error * TPI));
            pros::delay(20);
        }
    }
    else if (current_mode == TankDrive::auto_mode::TURN)
    {
        while (abs(turn_PID->error) > abs(error))
        {
            pros::delay(20);
        }
    }
    else if (current_mode == TankDrive::auto_mode::SWING)
    {
        while (abs(swing_PID->error) > abs(error))
        {
            pros::delay(20);
        }
    }
    // set_mode(TankDrive::auto_mode::DISABLE);
}