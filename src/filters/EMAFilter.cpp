#include "main.h"

EMAFilter::EMAFilter(double alpha)
{
    this->alpha = alpha;
}

double EMAFilter::filter(double input)
{
    output = this->alpha * input + (1.0 - this->alpha) * this->lastOutput;
    this->lastOutput = output;
    return output;
}

void EMAFilter::setGains(double alpha)
{
    this->alpha = alpha;
}