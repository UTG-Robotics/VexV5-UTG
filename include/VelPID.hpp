#pragma once
#include "main.h"

class VelPID
{
public:
    VelPID(double Kp, double Ki, double Kd, double Kf, double KfAddition, double alpha);
    double calculate(double targetRPM, double currentRPM);

private:
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double Kf = 0;
    double KfAddition = 0;
    double error = 0;
    double integral = 0;
    double lastError = 0;
    double derivative = 0;
    double output = 0;
    EMAFilter *dFilter;
};