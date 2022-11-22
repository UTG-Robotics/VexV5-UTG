#pragma once
#include "api.h"
class Piston
{
private:
    double _isExtended;
    pros::ADIDigitalOut *piston;

public:
    Piston(int port);
    bool isExtended();
    void setExtended(bool extended);
    bool toggle();
};