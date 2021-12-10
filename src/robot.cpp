#include "main.h"

void rotateToAngle(double targetAngle)
{
    pros::Motor front_right_mtr(10);
    pros::Motor front_left_mtr(1);
    pros::Motor back_right_mtr(20);
    pros::Motor back_left_mtr(11);

    double error = targetAngle - angle;
    double lastError = error;
    double speed = 0;
    double finishTimer = 0;

    double Kp = 0.6;
    double Ki = 0.01;
    double Kd = 0.05;

    double integral = 0;
    double derivative = 0;

    while (true)
    {
        error = angle - targetAngle;
        if (Ki != 0)
        {
            if (abs(error) < 1)
            {
                integral = 0;
            }
            if (abs(integral) < 300)
            {
                integral += error;
            }
            else
            {
                integral = 0;
            }
        }
        else
        {
            integral = 0;
        }

        derivative = error - lastError;
        lastError = error;

        speed = Kp * error + Ki * integral + Kd * derivative;
        speed = std::min(speed, 60.0);
        speed = std::max(speed, -60.0);
        pros::lcd::set_text(1, std::to_string(speed));

        front_left_mtr.move_velocity(speed);
        front_right_mtr.move_velocity(speed);
        back_left_mtr.move_velocity(speed);
        back_right_mtr.move_velocity(speed);
        if (abs(error) < 0.5)
        {
            finishTimer += 1;
        }
        else
        {
            finishTimer = 0;
        }

        if (finishTimer > 100)
        {
            break;
        }
        pros::delay(15);
    }
    pros::lcd::set_text(1, "Finished");

    front_left_mtr.move(0);
    front_right_mtr.move(0);
    back_left_mtr.move(0);
    back_right_mtr.move(0);
}

void driveForward(double inches)
{
    pros::Rotation leftEncoder(12);
    pros::Rotation rightEncoder(3);
    pros::Rotation sideEncoder(19);

    pros::Motor front_right_mtr(10);
    pros::Motor front_left_mtr(1);
    pros::Motor back_right_mtr(20);
    pros::Motor back_left_mtr(11);

    const double ENCODERTOINCHES = 0.00024;

    double start = sideEncoder.get_position() * ENCODERTOINCHES;

    double pos;
    double error = start - (start - inches);
    double lastError = error;
    double speed = 0;
    double finishTimer = 0;

    double Kp = 1.6;
    double Ki = 0.05;
    double Kd = 0.0;

    double integral = 0;
    double derivative = 0;

    while (true)
    {
        pos = sideEncoder.get_position() * ENCODERTOINCHES;
        error = pos - (start - inches);
        if (Ki != 0)
        {
            if (abs(error) < 0.5)
            {
                integral = 0;
            }
            if (abs(integral) < 72)
            {
                integral += error;
            }
            else
            {
                integral = 0;
            }
        }
        else
        {
            integral = 0;
        }

        derivative = error - lastError;
        lastError = error;

        speed = Kp * error + Ki * integral + Kd * derivative;
        speed = std::min(speed, 60.0);
        speed = std::max(speed, -60.0);
        pros::lcd::set_text(1, std::to_string(speed));

        front_left_mtr.move_velocity(speed);
        front_right_mtr.move_velocity(-speed);
        back_left_mtr.move_velocity(speed);
        back_right_mtr.move_velocity(-speed);
        if (abs(error) < 0.5)
        {
            finishTimer += 1;
        }
        else
        {
            finishTimer = 0;
        }

        if (finishTimer > 100)
        {
            break;
        }
        pros::lcd::set_text(3, "Finish Timer: " + std::to_string(finishTimer));
        pros::delay(10);
    }
    pros::lcd::set_text(2, "Finished");

    front_left_mtr.move(0);
    front_right_mtr.move(0);
    back_left_mtr.move(0);
    back_right_mtr.move(0);
}

void driveToPoint(double x, double y, double targetAngle)
{
    pros::Motor front_right_mtr(10);
    pros::Motor front_left_mtr(1);
    pros::Motor back_right_mtr(20);
    pros::Motor back_left_mtr(11);

    double errorSpeed = x - xPos;
    double errorAngle = targetAngle - angle;
    double lastErrorSpeed = errorSpeed;
    double lastErrorAngle = errorAngle;
    double speedSpeed = 0;
    double speedAngle = 0;
    double finishTimer = 0;

    double KpSpeed = 15;
    double KiSpeed = 0.00;
    double KdSpeed = 0.0;

    double KpAngle = 0.6;
    double KiAngle = 0.0;
    double KdAngle = 0.0;

    double integralSpeed = 0;
    double integralAngle = 0;

    double derivativeSpeed = 0;
    double derivativeAngle = 0;

    double multiplicationFactor = 0;
    double xSpeedPercent = 0;
    double ySpeedPercent = 0;

    double xDiff = 0;
    double yDiff = 0;

    while (true)
    {
        xDiff = xPos - x;
        yDiff = yPos - y;

        //Distance between the robot and the point
        errorSpeed = sqrt(xDiff * xDiff + yDiff * yDiff);
        errorAngle = targetAngle - angle;

        if (errorAngle > 180)
        {
            errorAngle -= 360;
        }
        else if (errorAngle < -180)
        {
            errorAngle += 360;
        }

        if (KiSpeed != 0)
        {
            if (abs(errorSpeed) < 1)
            {
                integralSpeed = 0;
            }
            if (abs(integralSpeed) < 300)
            {
                integralSpeed += errorSpeed;
            }
            else
            {
                integralSpeed = 0;
            }
        }
        else
        {
            integralSpeed = 0;
        }

        if (KiAngle != 0)
        {
            if (abs(errorAngle) < 1)
            {
                integralAngle = 0;
            }
            if (abs(integralAngle) < 300)
            {
                integralAngle += errorAngle;
            }
            else
            {
                integralAngle = 0;
            }
        }
        else
        {
            integralAngle = 0;
        }

        derivativeSpeed = errorSpeed - lastErrorSpeed;
        derivativeAngle = errorAngle - lastErrorAngle;

        lastErrorSpeed = errorSpeed;
        lastErrorAngle = errorAngle;

        speedSpeed = KpSpeed * errorSpeed + KiSpeed * integralSpeed + KdSpeed * derivativeSpeed;
        speedAngle = KpAngle * errorAngle + KiAngle * integralAngle + KdAngle * derivativeAngle;

        double speedLimit = 10;

        speedSpeed = std::min(speedSpeed, speedLimit);
        speedSpeed = std::max(speedSpeed, -speedLimit);

        speedAngle = std::min(speedAngle, speedLimit);
        speedAngle = std::max(speedAngle, -speedLimit);

        xSpeedPercent = sin(errorAngle * M_PI / 180);
        ySpeedPercent = cos(errorAngle * M_PI / 180);

        multiplicationFactor = 1 / MAX(abs(xSpeedPercent), abs(ySpeedPercent));

        if (xSpeedPercent < 0)
        {
            xSpeedPercent *= -multiplicationFactor;
        }
        else
        {
            xSpeedPercent *= multiplicationFactor;
        }

        if (ySpeedPercent < 0)
        {
            ySpeedPercent *= -multiplicationFactor;
        }
        else
        {
            ySpeedPercent *= multiplicationFactor;
        }

        ySpeedPercent *= speedSpeed;
        xSpeedPercent *= speedSpeed;

        front_right_mtr.move_velocity(-ySpeedPercent + xSpeedPercent + speedAngle);
        front_left_mtr.move_velocity(ySpeedPercent + xSpeedPercent + speedAngle);
        back_right_mtr.move_velocity(-ySpeedPercent + xSpeedPercent - speedAngle);
        back_left_mtr.move_velocity(ySpeedPercent + xSpeedPercent - speedAngle);

        pros::lcd::set_text(2, "Y: " + std::to_string(ySpeedPercent) + " X: " + std::to_string(xSpeedPercent));
        pros::lcd::set_text(3, "distErr: " + std::to_string(errorSpeed));
        if (abs(errorSpeed) < 0.5 && abs(errorAngle) < 0.5)
        {
            finishTimer += 1;
        }
        else
        {
            finishTimer = 0;
        }
        if (finishTimer > 100)
        {
            break;
        }
        pros::delay(15);
    }
    pros::lcd::set_text(1, "Finished");

    front_left_mtr.move(0);
    front_right_mtr.move(0);
    back_left_mtr.move(0);
    back_right_mtr.move(0);
}