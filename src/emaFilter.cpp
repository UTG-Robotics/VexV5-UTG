#include "main.h"

EMAFilter::EMAFilter(double alpha)
{
    this->alpha = alpha;
}

double EMAFilter::filter(double input)
{
    output = alpha * input + (1.0 - alpha) * lastOutput;
    lastOutput = output;
    return output;
}

void EMAFilter::setGains(double alpha)
{
    this->alpha = alpha;
}