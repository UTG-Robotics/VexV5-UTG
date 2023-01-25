#include "main.h"

PID::PID(double _Kp, double _Ki, double _Kd, double _KiStart)
{
    this->Kp = _Kp;
    this->Ki = _Ki;
    this->Kd = _Kd;
    this->KiStart = _KiStart;
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
        if (abs(this->error) < this->KiStart)
        {
            this->integral += this->error;
        }
        // if (lastError)
    }
    else
    {
        integral = 0;
    }

    derivative = this->error - this->lastError;
    return this->Kp * this->error + this->Ki * integral + this->Kd * derivative;
}

// void PID::setConstants(double p, double i, double d, double iStart)
// {
//     this->Kp = p;
//     this->Ki = i;
//     this->Kd = d;
//     this->KiStart = iStart;
// }
