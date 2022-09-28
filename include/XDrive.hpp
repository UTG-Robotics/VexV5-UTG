#pragma once
#include "main.h"

class XDrive
{
public:
    XDrive(pros::Motor *FR_mtr, pros::Motor *FL_mtr, pros::Motor *BR_mtr, pros::Motor *BL_mtr, double slew_rate);
    void arcade(int chOne, int chTwo, int chThree);
    void debug();

private:
    double slew_rate;
    pros::Motor *FR_mtr;
    pros::Motor *FL_mtr;
    pros::Motor *BR_mtr;
    pros::Motor *BL_mtr;
};