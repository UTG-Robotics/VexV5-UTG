#include "main.h"

XDrive::XDrive(pros::Motor *FR_mtr, pros::Motor *FL_mtr, pros::Motor *BR_mtr, pros::Motor *BL_mtr, double slew_rate)
{
    this->FR_mtr = FR_mtr;
    this->FL_mtr = FL_mtr;
    this->BR_mtr = BR_mtr;
    this->BL_mtr = BL_mtr;
    // Set brake modes
    this->FR_mtr->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    this->FL_mtr->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    this->BR_mtr->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    this->BL_mtr->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    this->slew_rate = slew_rate;
}

int sign(int num)
{
    if (num > 0)
        return 1;
    if (num < 0)
        return -1;
    return 0;
}

double slew(double current, double target, double rate)
{
    double change = std::min(std::pow(abs(target), 1 / 3) * (rate), abs(current - target));

    if (target < current)
    {
        current -= change;
    }
    else if (target > current)
    {
        current += change;
    }
    return current;
}
void XDrive::arcade(int x, int y, int rot)
{
    x = x;
    y = y;
    rot = rot;

    // this->FR_mtr->move(slew(FR_mtr->get_voltage() / 12000.0 * 127.0, -y + rot + x, this->slew_rate));
    // this->FL_mtr->move(slew(FL_mtr->get_voltage() / 12000.0 * 127.0, y + rot + x, this->slew_rate));
    // this->BR_mtr->move(slew(BR_mtr->get_voltage() / 12000.0 * 127.0, -y + rot - x, this->slew_rate));
    // this->BL_mtr->move(slew(BL_mtr->get_voltage() / 12000.0 * 127.0, y + rot - x, this->slew_rate));

    int speeds[4] = {-y + rot + x, y + rot + x, -y + rot - x, y + rot - x};

    // int max = 0;
    // for (int i = 0; i < 4; i++)
    // {
    //     if (abs(speeds[i]) > max)
    //     {
    //         max = abs(speeds[i]);
    //     }
    // }

    // if (max > 127)
    // {
    //     for (int i = 0; i < 4; i++)
    //     {
    //         speeds[i] = speeds[i] / max * 127;
    //     }
    // }
    this->FR_mtr->move(speeds[0]);
    this->FL_mtr->move(speeds[1]);
    this->BR_mtr->move(speeds[2]);
    this->BL_mtr->move(speeds[3]);
    // this->FR_mtr->move(-y + rot + x);
    // this->FL_mtr->move(y + rot + x);
    // this->BR_mtr->move(-y + rot - x);
    // this->BL_mtr->move(y + rot - x);
}

void XDrive::debug()
{
    printf("%f,%f,%f,%f\n", this->FR_mtr->get_actual_velocity(), this->FL_mtr->get_actual_velocity(), this->BR_mtr->get_actual_velocity(), this->BL_mtr->get_actual_velocity());
}

void XDrive::driveToPoint(double x, double y, double targetAngle, double maxSpeed, int timeout)
{
    bool isWithinRange = false;

    double errorSpeed;
    double errorAngle;

    double speedSpeed = 0;
    double speedAngle = 0;
    double finishTimer = 0;

    double speedLimit = maxSpeed;

    double driveAngle;
    double maxRatio;
    double xRatio;
    double yRatio;

    double xPowerPercentage;
    double yPowerPercentage;

    double xDiff = 0;
    double yDiff = 0;

    int startTime = pros::millis();

    // Initialize PID constants

    PID speedPID = PID(0.13, 0.001, 0.4, 5);
    PID anglePID = PID(0.015, 0.001, 0.005, 5);
    // PID speedPID = PID(0, 0, 0, 0);
    // PID anglePID = PID(0, 0, 0, 0);
    // printf("time,error,output,xPow,yPow");
    while (true)
    {

        xDiff = x - (xPos + this->xPosStart);
        yDiff = y - (yPos + this->yPosStart);

        // Calculate distance between the robot and the point
        errorSpeed = sqrt(xDiff * xDiff + yDiff * yDiff);
        errorAngle = targetAngle - (angle * 180 / M_PI + this->angleStart);

        speedSpeed = slew(speedSpeed, speedPID.calculate(errorSpeed), 0.05);
        speedAngle = slew(speedAngle, anglePID.calculate(errorAngle), 0.05);

        speedSpeed = std::clamp(speedSpeed * 127.0, -speedLimit, speedLimit) / 127.0;
        speedAngle = std::clamp(speedAngle * 127.0, -speedLimit, speedLimit) / 127.0;

        // Calculate angle needed to drive at to go to the point
        driveAngle = 2 * M_PI - (atan2(x - (xPos + this->xPosStart), y - (yPos + this->yPosStart)) - angle - M_PI / 2);
        // printf("\ndriveAngle: %f", driveAngle * 180 / M_PI);
        // Calculate how much to move each set of opposite wheels to move at that angle
        xRatio = -cos(driveAngle + (M_PI / 4));
        yRatio = sin(driveAngle + (M_PI / 4));

        // printf("\ndriveAngle: %f", std::fmod(driveAngle * 180 / M_PI, 360));
        // printf("\nxRatio: %f yRatio: %f", xRatio, yRatio);

        // Normalize values to maximum of 1
        maxRatio = std::max(abs(xRatio), abs(yRatio));
        xPowerPercentage = (xRatio / maxRatio);
        yPowerPercentage = (yRatio / maxRatio);

        // Move at angle while rotating
        this->FR_mtr->move(-xPowerPercentage * (speedSpeed * 127) + (speedAngle * 127));
        this->FL_mtr->move(yPowerPercentage * (speedSpeed * 127) + (speedAngle * 127));
        this->BR_mtr->move(-yPowerPercentage * (speedSpeed * 127) + (speedAngle * 127));
        this->BL_mtr->move(xPowerPercentage * (speedSpeed * 127) + (speedAngle * 127));

        isWithinRange = (abs(errorSpeed) < 0.5 && abs(errorAngle) < 0.5);
        // If the error is within an acceptable margin or timeout is over, start timer
        // printf("\n%d,%f,%f,%f,%f", pros::millis(), errorSpeed, speedSpeed, xPowerPercentage * (speedSpeed * 127) + (speedAngle * 127), yPowerPercentage * (speedSpeed * 127) + (speedAngle * 127));
        // printf("\n%d,%f,%f", pros::millis(), errorAngle, speedAngle);
        if ((isWithinRange || (pros::millis() - startTime) > timeout))
        {
            finishTimer += 1;
        }
        else
        {
            finishTimer = 0;
        }
        if (finishTimer > 30)
        {
            break;
        }
        pros::delay(10);
    }

    // Stop robot
    speedPID.clear();
    anglePID.clear();
    this->FR_mtr->move(0);
    this->FL_mtr->move(0);
    this->BR_mtr->move(0);
    this->BL_mtr->move(0);
}

void XDrive::rotate(double targetAngle, double maxSpeed, int timeout)
{
    bool isWithinRange = false;

    double errorAngle;

    double speedAngle = 0;
    double finishTimer = 0;

    // Initialize PID constants
    PID anglePID = PID(0.01, 0.00015, 0.1);

    double speedLimit = maxSpeed;

    int startTime = pros::millis();
    while (true)
    {
        errorAngle = targetAngle - angle * 180 / M_PI;

        speedAngle = anglePID.calculate(errorAngle);
        // speedAngle = 0;

        speedAngle = std::clamp(speedAngle * 127.0, -speedLimit, speedLimit) / 127.0;

        // Calculate how much to move each set of opposite wheels to move at that angle

        this->FR_mtr->move(speedAngle * 127);
        this->FL_mtr->move(speedAngle * 127);
        this->BR_mtr->move(speedAngle * 127);
        this->BL_mtr->move(speedAngle * 127);

        isWithinRange = (abs(errorAngle) < 1);

        // If the error is within an acceptable margin or timeout is over, start timer
        if ((isWithinRange || (pros::millis() - startTime) > timeout))
        {
            finishTimer += 1;
        }
        else
        {
            finishTimer = 0;
        }
        if (finishTimer > 30)
        {
            break;
        }
        pros::delay(10);
    }
    // Stop robot
    anglePID.clear();
    this->FR_mtr->move(0);
    this->FL_mtr->move(0);
    this->BR_mtr->move(0);
    this->BL_mtr->move(0);
}
void XDrive::setStartPos(double _x, double _y, double _angle)
{
    xPos = _x;
    yPos = _y;
    angle = _angle;
}