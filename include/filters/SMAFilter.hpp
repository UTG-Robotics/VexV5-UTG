#pragma once
#include "main.h"
#include <deque>

class SMAFilter
{
private:
    int length = 1;
    std::deque<double> values;

public:
    SMAFilter(int length);
    double filter(double input);
    void setLength(int length);
};
