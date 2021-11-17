#include "main.h"

void rotateToAngle(double angle)
{
    pros::Motor front_right_mtr(10);
    pros::Motor front_left_mtr(1);
    pros::Motor back_right_mtr(20);
    pros::Motor back_left_mtr(11);

    double error = angle - theta;
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
        error = theta - angle;
        if (Ki != 0)
        {
            if (abs(error) < 0.1)
            {
                integral = 0;
            }
            if (abs(integral) < 1000)
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