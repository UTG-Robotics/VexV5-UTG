#include "main.h"

Indexer::Indexer(pros::Motor *motor)
{
    this->motor = motor;
    this->motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    this->motor->set_reversed(true);
    this->motor->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    this->task = new pros::Task(this->taskFn, (void *)this);
}

void Indexer::run()
{
    while (true)
    {
        if (this->isShooting)
        {
            if (this->motor->get_position() - 360 > -20)
            {
                this->hasShot = true;
                this->isShooting = false;
            }
            // if (this->hasShot && this->motor->get_position() < 5)
            // {
            //     this->isShooting = false;
            //     this->hasShot = false;
            //     // this->motor->move(-20);
            // }
        }
        pros::delay(20);
    }
}

void Indexer::shoot()
{
    if (!this->isShooting)
    {
        this->isShooting = true;
        this->motor->tare_position();
        this->motor->move_absolute(360, 200);
    }
}

void Indexer::taskFn(void *param)
{
    ((Indexer *)param)->run();
}