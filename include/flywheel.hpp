#pragma once
#include "main.h"

class Flywheel
{
public:
    Flywheel(pros::Motor *motor, VelPID *pid, EMAFilter *filter, double gearRatio, double motorSlew);
    void run();
    static void taskFn(void *param);
    void setTargetRPM(double rpm);
    double getTargetRPM();
    double getCurrentRPM();
    double getAverageRPM();
    void waitUntilReady();
    void updatePID(VelPID *pid);
    bool isShot = false;
    bool isRecovering = false;

private:
    pros::Motor *flywheelMotor;
    VelPID *pid;
    EMAFilter *rpmFilter;
    pros::Task *task;

    double averageRPM = 0;
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