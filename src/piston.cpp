#include "main.h"

Piston::Piston(int port)
{
    piston = new pros::ADIDigitalOut(port);
}

void Piston::setExtended(bool extended)
{
    this->_isExtended = extended;
    piston->set_value(extended);
}

bool Piston::isExtended()
{
    return this->_isExtended;
}

bool Piston::toggle()
{
    Piston::setExtended(!this->_isExtended);
    return this->_isExtended;
}