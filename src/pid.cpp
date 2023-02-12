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
    this->error = 0;
    this->stopTimer = 0;
    this->timeOutTimer = 0;
    return;
}
void PID::set_target(double target)
{
    this->clear();
    this->target = target;
    this->error = 999999;
    return;
}

double PID::calculate(double current)
{
    this->lastError = this->error;
    this->error = this->target - current;

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

bool PID::exit(double tolerance, double stop_time, double timeout_time)
{
    if (abs(this->error) < tolerance)
    {
        this->stopTimer += 0.02;
    }
    else
    {
        this->stopTimer = 0;
    }
    this->timeOutTimer += 0.2;
    return this->stopTimer > stop_time || this->timeOutTimer > timeout_time;
}

// void PID::setConstants(double p, double i, double d, double iStart)
// {
//     this->Kp = p;
//     this->Ki = i;
//     this->Kd = d;
//     this->KiStart = iStart;
// }
