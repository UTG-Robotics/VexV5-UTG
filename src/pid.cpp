#include "main.h"

PID::PID(double _Kp, double _Ki, double _Kd)
{
    this->Kp = _Kp;
    this->Ki = _Ki;
    this->Kd = _Kd;
}

void PID::clear()
{
    this->integral = 0;
    this->lastError = 0;
    return;
}

double PID::calculate(double error)
{
    this->lastError = this->error;
    this->error = error;

    if (this->Ki != 0)
    {
        integral += this->error;
        if (std::signbit(this->error) != std::signbit(this->lastError))
        {
            this->integral *= 0.75;
        }
    }
    else
    {
        integral = 0;
    }

    derivative = this->error - this->lastError;
    return this->Kp * this->error + this->Ki * integral + this->Kd * derivative;
}