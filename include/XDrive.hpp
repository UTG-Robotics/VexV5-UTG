#pragma once
#include "main.h"

class XDrive
{
public:
    XDrive(pros::Motor *FR_mtr, pros::Motor *FL_mtr, pros::Motor *BR_mtr, pros::Motor *BL_mtr, double slew_rate);
    void arcade(int chOne, int chTwo, int chThree);
    void driveToPoint(double x, double y, double targetAngle, double maxSpeed, int timeout);
    void rotate(double targetAngle, double maxSpeed, int timeout);
    void setStartPos(double x, double y, double angle);
    void debug();

private:
    double slew_rate;
    double xPosStart;
    double yPosStart;
    double angleStart;

    pros::Motor *FR_mtr;
    pros::Motor *FL_mtr;
    pros::Motor *BR_mtr;
    pros::Motor *BL_mtr;
};