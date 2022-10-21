#pragma once
#include "main.h"

class Indexer
{
public:
    Indexer(pros::Motor *motor);
    void run();
    void shoot();
    static void taskFn(void *param);

private:
    pros::Motor *motor;
    pros::Task *task;
    bool isShooting = false;
};