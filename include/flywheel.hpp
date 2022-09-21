#pragma once
#include "main.h"

class Flywheel
{
public:
    Flywheel(int motor, VelPID *pid, EMAFilter *filter, double gearRatio, double motorSlew);
    void run();
    static void taskFn(void *param);
    void setTargetRPM(double rpm);
    double getTargetRPM();
    double getCurrentRPM();
    bool isShot = false;
    bool isRecovering = false;

private:
    pros::Motor *flywheelMotor;
    VelPID *pid;
    EMAFilter *rpmFilter;
    pros::Task *task;

    int oldTime = 0;
    double targetRPM = 0;
    double currentRPM = 0;
    double oldRPM = 0;
    double output = 0;
    double error = 0;
    double acceleration = 0;
    double gearRatio = 0;
    double motorSlew = 0;
    double lastOutput = 0;
    int lastShotTime = 0;
    double lastShotSpeed = 0;
    double oldTarget = 0;
};