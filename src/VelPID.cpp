#include "main.h"

VelPID::VelPID(double Kp, double Ki, double Kd, double Kf, double KfAddition, double alpha, bool isAuto)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->Kf = Kf;
    this->KfAddition = KfAddition;
    this->isAuto = isAuto;
    this->dFilter = new EMAFilter(alpha);
}

double VelPID::calculate(double targetRPM, double currentRPM)
{
    double threshold = 100;
    error = targetRPM - currentRPM;
    if (abs(error) < 300)
    {
        integral += error;
    }

    derivative = error - lastError;
    lastError = error;

    derivative = dFilter->filter(derivative);

    output = Kp * error + Ki * integral + Kd * derivative + Kf * targetRPM + KfAddition;

    if (error > threshold)
    {
        output = 12000;
    }
    else if (error < -threshold * 2)
    {
        output = -12000;
    }

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