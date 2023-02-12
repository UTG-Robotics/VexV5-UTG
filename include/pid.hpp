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
    double timeOutTimer = 0;
    double stopTimer = 0;

public:
    double error;
    double target;
    PID(double _Ki, double _Kp, double _Kd, double _KiStart = 999999);
    void set_target(double target);
    void set_constants(double p, double i, double d, double iStart);
    double calculate(double error);
    bool exit(double tolerance, double stop_time, double timeout_time);
    void clear();
};