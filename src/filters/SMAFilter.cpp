#include "main.h"
#include <numeric>

SMAFilter::SMAFilter(int length)
{
    this->length = length;
}

double SMAFilter::filter(double input)
{
    if (this->values.size() >= this->length)
    {
        this->values.pop_back();
    }
    this->values.push_front(input);
    double sum = std::accumulate(values.begin(), values.end(), 0);
    return sum / this->values.size();
}

void SMAFilter::setLength(int length)
{
    this->length = length;
}