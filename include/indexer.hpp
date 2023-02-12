#pragma once
#include "main.h"

class Indexer
{
public:
    Indexer(pros::Motor *motor, pros::ADIPotentiometer *potentiometer);
    void run();
    void shoot(bool is_blocking = false);
    static void taskFn(void *param);

private:
    pros::Motor *motor;
    pros::ADIPotentiometer *potentiometer;
    pros::Task *task;
    PID *indexerPID;
    bool isShooting = false;
    bool hasShot = false;
};