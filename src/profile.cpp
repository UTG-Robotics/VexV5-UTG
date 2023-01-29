#include "main.h"
std::vector<double> generateProfile(double start, double target, double maxVel, double maxAccel, double maxJerk, double dt)
{
    target -= start;
    std::vector<double> output;
    double curTime = 0;

    double curPos = 0;
    double curVel = 0;
    double curAccel = 0;
    double curJerk = 0;
    int i = 0;
    printf("start\n");
    while (true)
    {
        curTime += dt;
        if (target / maxVel < (maxAccel / maxJerk + maxVel / maxAccel))
        {
            double b = maxAccel / maxJerk;
            maxVel = maxAccel * (-b + sqrt((b * b) + 4 * target / maxAccel)) / 2.0;
        }

        if (maxVel * maxVel < maxAccel * maxAccel)
        {
            maxAccel = sqrt(maxJerk * maxVel);
        }
        double t1 = maxAccel / maxJerk;
        double t2 = maxVel / maxAccel - maxAccel / maxJerk;
        double t3 = target / maxVel - maxVel / maxAccel - maxAccel / maxJerk;
        double t4 = t1;
        double t5 = t2;
        double K = 2.0 * target / (t1 * (t1 + t2) * (t5 + 2.0 * t4 + 2.0 * t3 + 2.0 * t1 + t2));

        if (curTime < t1)
        {
            curJerk = K;
        }
        else if (curTime < (t1 + t2))
        {
            curJerk = 0;
        }
        else if (curTime < (2 * t1 + t2))
        {
            curJerk = -K;
        }
        else if (curTime < (2 * t1 + t2 + t3))
        {
            curJerk = 0;
        }
        else if (curTime < (2 * t1 + t2 + t3 + t4))
        {
            curJerk = -K;
        }
        else if (curTime < (2 * t1 + t2 + t3 + t4 + t5))
        {
            curJerk = 0;
        }
        else if (curTime < (2 * t1 + t2 + t3 + 2 * t4 + t5))
        {
            curJerk = K;
        }
        else
        {
            curJerk = 0;
        }
        curAccel += curJerk * dt;
        curVel += curAccel * dt;
        curPos += curVel * dt;

        output.push_back(curVel);

        if (curVel < 0.01 && curAccel < 0.01 && curJerk == 0)
        {
            break;
        }
        // i++;
        // if (i % 100 == 0)
        // {
        //     printf("Step %i - CurVel: %f, CurAccel: %f, CurJerk: %f\n", i, curVel, curAccel, curJerk);
        // }
        pros::delay(1);
    }
    printf("function size: %i\n", output.size());
    std::vector<double> profile;
    for (int i = 0; i < output.size(); i += (int)(0.02 / dt))
    {
        profile.push_back(output.at(i));
    }
    return profile;
}