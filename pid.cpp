class PID
{
private:
    double Ki = 0;
    double Kp = 0;
    double Kd = 0;
    double lastError = 0;
    double derivative = 0;
    double integral = 0;

public:
    double error;
    PID(double _Ki, double _Kp, double _Kd)
    {
        this->Ki = _Ki;
        this->Kp = _Kp;
        this->Kd = _Kd;
    }
    double calculate(double _error);
};

double PID::calculate(double _error)
{
    this->lastError = this->error;
    this->error = _error;

    if (this->Ki != 0)
    {
        if (abs(this->error) > 1)
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

    return this->Kp * this->error + this->Ki * integral + this->Kd * derivative;
}