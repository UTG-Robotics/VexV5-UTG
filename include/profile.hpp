#pragma once
#include <vector>

class SetPoint
{
public:
    double position;
    double velocity;
    double acceleration;
    double jerk;
    double time;
    SetPoint(double position, double velocity, double acceleration, double jerk, double time)
    {
        this->position = position;
        this->velocity = velocity;
        this->acceleration = acceleration;
        this->jerk = jerk;
        this->time = time;
    }
};

std::vector<SetPoint> generateProfile(double start, double target, double maxVel, double maxAccel, double maxJerk, double dt, double outputTime = 0.02);
