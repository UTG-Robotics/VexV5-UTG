#include "main.h"

PID::PID(double _Ki, double _Kp, double _Kd)
{
    this->Ki = _Ki;
    this->Kp = _Kp;
    this->Kd = _Kd;
}

double PID::calculate(double _error)
{
    this->lastError = this->error;
    this->error = _error;

    if (this->Ki != 0)
    {
        if (abs(this->error) > 1)
        {
            integral = 0;
        }
        else
        {
            integral += this->error;
        }
    }
    else
    {
        integral = 0;
    }

    derivative = this->error - this->lastError;

    return this->Kp * this->error + this->Ki * integral + this->Kd * derivative;
}