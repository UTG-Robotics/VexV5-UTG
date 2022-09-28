#include "main.h"

XDrive::XDrive(pros::Motor *FR_mtr, pros::Motor *FL_mtr, pros::Motor *BR_mtr, pros::Motor *BL_mtr, double slew_rate)
{
    this->FR_mtr = FR_mtr;
    this->FL_mtr = FL_mtr;
    this->BR_mtr = BR_mtr;
    this->BL_mtr = BL_mtr;
    this->slew_rate = slew_rate;
}

int sign(int num)
{
    if (num > 0)
        return 1;
    if (num < 0)
        return -1;
    return 0;
}

double slew(double current, double target, double rate)
{
    double change = std::min(std::pow(abs(target), 1 / 3) * (rate), abs(current - target));

    if (target < current)
    {
        current -= change;
    }
    else if (target > current)
    {
        current += change;
    }
    return current;
}
void XDrive::arcade(int x, int y, int rot)
{
    x = x;
    y = y;
    rot = rot;

    this->FR_mtr->move(slew(FR_mtr->get_voltage() / 12000.0 * 127.0, -y + rot + x, this->slew_rate));
    this->FL_mtr->move(slew(FL_mtr->get_voltage() / 12000.0 * 127.0, y + rot + x, this->slew_rate));
    this->BR_mtr->move(slew(BR_mtr->get_voltage() / 12000.0 * 127.0, -y + rot - x, this->slew_rate));
    this->BL_mtr->move(slew(BL_mtr->get_voltage() / 12000.0 * 127.0, y + rot - x, this->slew_rate));
}

void XDrive::debug()
{
    printf("%f,%f,%f,%f\n", this->FR_mtr->get_actual_velocity(), this->FL_mtr->get_actual_velocity(), this->BR_mtr->get_actual_velocity(), this->BL_mtr->get_actual_velocity());
}