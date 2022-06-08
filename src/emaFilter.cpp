#include "emaFilter.hpp"

emaFilter::emaFilter(double alpha)
{
    this->alpha = alpha;
}

double emaFilter::filter(double input)
{
    output = alpha * input + (1.0 - alpha) * lastOutput;
    lastOutput = output;
    return output;
}

void emaFilter::setGains(double alpha) { emaFilter::alpha = alpha; }