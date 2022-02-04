#include "main.h"

void rotateToAngle(double targetAngle)
{
    pros::Motor front_right_mtr(6);
    pros::Motor front_left_mtr(5);
    pros::Motor back_right_mtr(16);
    pros::Motor back_left_mtr(15);

    double error = targetAngle - angle * 180 / M_PI;
    double lastError = error;
    double speed = 0;
    double finishTimer = 0;

    double Kp = 1;
    double Ki = 0.03;
    double Kd = 0.05;

    double integral = 0;
    double derivative = 0;

    while (true)
    {
        error = angle * 180 / M_PI - targetAngle;
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

        front_left_mtr.move_velocity(-speed);
        front_right_mtr.move_velocity(-speed);
        back_left_mtr.move_velocity(-speed);
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
    pros::Rotation leftEncoder(11);
    pros::Rotation rightEncoder(20);
    pros::Rotation sideEncoder(7);

    pros::Motor front_right_mtr(6);
    pros::Motor front_left_mtr(5);
    pros::Motor back_right_mtr(16);
    pros::Motor back_left_mtr(15);

    const double ENCODERTOINCHES = 0.00024;

    double start = yPos;

    double pos;
    double error = start - (start + inches);
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
        error = yPos - (start + inches);
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
        pros::lcd::set_text(1, std::to_string(error));

        front_left_mtr.move_velocity(-speed);
        front_right_mtr.move_velocity(speed);
        back_left_mtr.move_velocity(-speed);
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
    // x *= -1;
    // y *= -1;
    pros::Motor front_right_mtr(6);
    pros::Motor front_left_mtr(5);
    pros::Motor back_right_mtr(16);
    pros::Motor back_left_mtr(15);

    double errorSpeed;
    double errorAngle;
    double lastErrorSpeed;
    double lastErrorAngle;
    double speedSpeed = 0;
    double speedAngle = 0;
    double finishTimer = 0;

    double KpSpeed = 15;
    double KiSpeed = 0.00;
    double KdSpeed = 0.0;

    double KpAngle = 1;
    double KiAngle = 0.0;
    double KdAngle = 0.0;

    double speedLimit = 60;

    double driveAngle;
    double maxRatio;
    double xRatio;
    double yRatio;

    double xPowerPercentage;
    double yPowerPercentage;

    double integralSpeed = 0;
    double integralAngle = 0;

    double derivativeSpeed = 0;
    double derivativeAngle = 0;

    double xDiff = 0;
    double yDiff = 0;

    int i = 0;

    while (true)
    {
        xDiff = x - xPos;
        yDiff = y - yPos;

        //Distance between the robot and the point
        errorSpeed = sqrt(xDiff * xDiff + yDiff * yDiff);
        errorAngle = targetAngle - angle * 180 / M_PI;

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

        speedSpeed = std::min(speedSpeed, speedLimit);
        speedSpeed = std::max(speedSpeed, -speedLimit);

        speedAngle = std::min(speedAngle, speedLimit);
        speedAngle = std::max(speedAngle, -speedLimit);

        driveAngle = atan2(x - xPos, y - yPos) + angle + M_PI / 2;

        xRatio = -cos(driveAngle + (M_PI / 4));
        yRatio = sin(driveAngle + (M_PI / 4));

        maxRatio = std::max(abs(xRatio), abs(yRatio));

        xPowerPercentage = (xRatio / maxRatio);
        yPowerPercentage = (yRatio / maxRatio);

        front_right_mtr.move_velocity(-xPowerPercentage * speedSpeed + speedAngle);
        front_left_mtr.move_velocity(yPowerPercentage * speedSpeed + speedAngle);
        back_right_mtr.move_velocity(-yPowerPercentage * speedSpeed + speedAngle);
        back_left_mtr.move_velocity(xPowerPercentage * speedSpeed + speedAngle);

        // front_right_mtr.move_velocity(30);
        // front_left_mtr.move_velocity(30);
        // back_right_mtr.move_velocity(30);
        // back_left_mtr.move_velocity(30);

        pros::lcd::set_text(4, "distErr: " + std::to_string(errorSpeed));

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
        i++;
        if (i % 5 == 0)
        {
            printf("%f %f %f %f %f %f %f %f %f\n", xPowerPercentage, yPowerPercentage, xDiff, yDiff, errorSpeed, driveAngle, xPos, yPos, angle);
        }

        pros::delay(15);
    }
    pros::lcd::set_text(1, "Finished");

    front_left_mtr.move(0);
    front_right_mtr.move(0);
    back_left_mtr.move(0);
    back_right_mtr.move(0);
}