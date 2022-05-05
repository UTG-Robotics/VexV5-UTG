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

double PID::calculate(double _error)
{
    this->lastError = this->error;
    this->error = _error;

    if (this->Ki != 0)
    {
        if (abs(integral) > 200)
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
    if (this->Ki == 0.09)
    {
        // printf("2: %f %f %f", this->error, this->Kp, this->error * this->Kp);
    }
    return this->Kp * this->error + this->Ki * integral + this->Kd * derivative;
}