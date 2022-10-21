#include "main.h"

Indexer::Indexer(pros::Motor *motor)
{
    this->motor = motor;
    this->motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    this->motor->set_reversed(true);

    this->task = new pros::Task(this->taskFn, (void *)this);
}

void Indexer::run()
{
    while (true)
    {
        if (this->isShooting)
        {
            if (this->motor->get_position() - 100 > -20)
            {
                this->isShooting = false;
                this->motor->move_absolute(0, 200);
            }
        }
        pros::delay(20);
    }
}

void Indexer::shoot()
{
    if (!this->isShooting)
    {
        this->motor->tare_position();
        this->motor->move_absolute(100, 200);
        this->isShooting = true;
    }
}

void Indexer::taskFn(void *param)
{
    ((Indexer *)param)->run();
}