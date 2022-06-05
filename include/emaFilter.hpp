#pragma once
class emaFilter
{
private:
    double alpha = 1.0;
    double output = 0.0;
    double lastOutput = 0.0;

public:
    emaFilter(double);
    double filter(double);
    void setGains(double);
};
