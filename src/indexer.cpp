#include "main.h"

Indexer::Indexer(pros::Motor *motor, pros::ADIPotentiometer *potentiometer)
{
    this->motor = motor;
    this->motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    this->motor->set_reversed(true);
    this->motor->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    this->potentiometer = potentiometer;
    this->indexerPID = new PID(0.07, 0, 0.01, 0);
    indexerPID->set_target(870);
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
                indexerPID->set_target(870);
            }
        }
        else
        {
            this->motor->move(-indexerPID->calculate(this->potentiometer->get_value()));
        }
        pros::delay(20);
    }
}

void Indexer::shoot(bool is_blocking)
{
    if (!this->isShooting)
    {
        this->isShooting = true;
        this->motor->tare_position();
        this->motor->move_absolute(360, 200);
    }
    if (is_blocking)
    {
        while (this->isShooting)
        {
            pros::delay(20);
        }
    }
}

void Indexer::taskFn(void *param)
{
    ((Indexer *)param)->run();
}