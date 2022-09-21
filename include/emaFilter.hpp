#pragma once
class EMAFilter
{
private:
    double alpha = 1.0;
    double output = 0.0;
    double lastOutput = 0.0;

public:
    EMAFilter(double);
    double filter(double);
    void setGains(double);
};
