class PID
{
private:
    double Ki;
    double Kp;
    double Kd;
    double lastError;
    double derivative;
    double integral;

public:
    double error;
    PID(double _Ki, double _Kp, double _Kd);
    double calculate(double _error);
    void clear();
};