#include "main.h"
double targetX = 0;
double targetY = 0;
double targetAngleGlobal = 0;

void rotateToAngle(double targetAngle)
{
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
            if (abs(error) < 0.3)
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
        speed = std::min(speed, 127.0);
        speed = std::max(speed, -127.0);
        pros::lcd::set_text(1, std::to_string(speed));

        front_left_mtr.move(-speed);
        front_right_mtr.move(-speed);
        back_left_mtr.move(-speed);
        back_right_mtr.move(-speed);
        if (abs(error) < 0.3)
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

        printf("%f\n", error);
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

        front_left_mtr.move(-speed);
        front_right_mtr.move(speed);
        back_left_mtr.move(-speed);
        back_right_mtr.move(speed);
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

void driveToPoint(double x, double y, double targetAngle, double maxSpeed, int timeout)
{
    bool isWithinRange = false;
    bool movingTarget = false;
    double oldTargetX = 0;
    double oldTargetY = 0;
    double oldTargetAngleGlobal = 0;

    targetX = x;
    targetY = y;
    targetAngleGlobal = targetAngle;

    double errorSpeed;
    double errorAngle;

    double speedSpeed = 0;
    double speedAngle = 0;
    double finishTimer = 0;

    // Initialize PID constants

    PID speedPID = PID(0.09, 0.001, 0.3);
    PID anglePID = PID(0.01, 0, 0);

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

    while (true)
    {
        if (targetX != oldTargetX || targetY != oldTargetY || targetAngleGlobal != oldTargetAngleGlobal)
        {
            movingTarget = true;
        }
        else
        {
            movingTarget = false;
        }

        xDiff = targetX - xPos;
        yDiff = targetY - yPos;
        oldTargetX = targetX;
        oldTargetY = targetY;
        oldTargetAngleGlobal = targetAngleGlobal;

        // Calculate distance between the robot and the point
        errorSpeed = sqrt(xDiff * xDiff + yDiff * yDiff);
        errorAngle = targetAngleGlobal - angle * 180 / M_PI;

        speedSpeed = speedPID.calculate(errorSpeed);
        speedAngle = anglePID.calculate(errorAngle);

        speedSpeed = std::clamp(speedSpeed * 127, speedLimit, -speedLimit) / 127;
        speedAngle = std::clamp(speedAngle * 127, speedLimit, -speedLimit) / 127;

        // Calculate angle needed to drive at to go to the point
        driveAngle = atan2(targetX - xPos, targetY - yPos) + angle + M_PI / 2;

        // Calculate how much to move each set of opposite wheels to move at that angle
        xRatio = -cos(driveAngle + (M_PI / 4));
        yRatio = sin(driveAngle + (M_PI / 4));

        // Normalize values to maximum of 1
        maxRatio = std::max(abs(xRatio), abs(yRatio));
        xPowerPercentage = (xRatio / maxRatio);
        yPowerPercentage = (yRatio / maxRatio);

        // Move at angle while rotating
        front_right_mtr.move(-xPowerPercentage * (speedSpeed * 127) + (speedAngle * 127));
        front_left_mtr.move(yPowerPercentage * (speedSpeed * 127) + (speedAngle * 127));
        back_right_mtr.move(-yPowerPercentage * (speedSpeed * 127) + (speedAngle * 127));
        back_left_mtr.move(xPowerPercentage * (speedSpeed * 127) + (speedAngle * 127));

        isWithinRange = (abs(errorSpeed) < 0.5 && abs(errorAngle) < 0.5);

        // If the error is within an acceptable margin or timeout is over, start timer
        if ((isWithinRange || (pros::millis() - startTime) > timeout) && !movingTarget)
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
        pros::delay(10);
    }

    // Stop robot
    front_left_mtr.move(0);
    front_right_mtr.move(0);
    back_left_mtr.move(0);
    back_right_mtr.move(0);
}

void move_relative_blocking(pros::Motor &targetMotor, int amount, int rpm, int timeOut)
{
    int startTime = pros::millis();
    int targetPos = targetMotor.get_position() + amount;
    targetMotor.move_relative(amount, rpm);
    while (abs(targetPos - targetMotor.get_position()) > 5 && (pros::millis() - startTime) < timeOut)
    {
        pros::delay(2);
    }
}

double clampf(double value, double min, double max)
{
    if (value > max)
    {
        return max;
    }
    else if (value < min)
    {
        return min;
    }
    else
    {
        return value;
    }
}

double slew(double target, double current, double slewRate)
{
    printf("pre: target: %f current: %f slewRate: %f\n", target, current, slewRate);
    double difference = target - current;
    int sign = 0;

    if (difference > 0)
    {
        sign = 1;
    }
    else if (difference < 0)
    {
        sign = -1;
    }
    printf("first: sign: %i target: %f current: %f slewRate: %f\n", sign, target, current, slewRate);
    current += slewRate * sign;
    // if (current > )
    // {
    //     current = target;
    // }
    printf("second: sign: %i target: %f current: %f slewRate: %f\n", sign, target, current, slewRate);
    return current;
}