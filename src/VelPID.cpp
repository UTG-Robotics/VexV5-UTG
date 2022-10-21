#include "main.h"

VelPID::VelPID(double Kp, double Ki, double Kd, double Kf, double KfAddition, double alpha)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->Kf = Kf;
    this->KfAddition = KfAddition;
    this->dFilter = new EMAFilter(alpha);
}

double VelPID::calculate(double targetRPM, double currentRPM)
{
    error = targetRPM - currentRPM;
    if (abs(integral) < 15000)
    {
        integral += error;
    }
    // if (std::signbit(error) != std::signbit(lastError))
    // {
    //     integral *= 0.7;
    // }

    derivative = error - lastError;
    lastError = error;

    derivative = dFilter->filter(derivative);

    output = Kp * error + Ki * integral + Kd * derivative + Kf * targetRPM + KfAddition;
    if (output > 12000)
    {
        output = 12000;
    }
    else if (output < -12000)
    {
        output = -12000;
    }
    return output;
}