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
    void set_constants(double p, double i, double d, double iStart);
    double calculate(double error);
    void clear();
};