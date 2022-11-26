#pragma once
class PID
{
private:
    double Ki;
    double Kp;
    double Kd;
    double KiStart;
    double lastError;
    double derivative;
    double integral;

public:
    double error;
    PID(double _Ki, double _Kp, double _Kd, double _KiStart = 999999);
    double calculate(double error);
    void clear();
};