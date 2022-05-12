#include "main.h"

void armTask(void *param)
{
    Arm *arm = (Arm *)param;
    double speed = 0;
    while (true)
    {
        printf("%f\n", arm->getAngle());
        if (arm->isEnabled)
        {
            arm->error = arm->asyncTargetPos - arm->getAngle();
            speed = arm->updatePID(arm->error);
            speed = std::clamp(speed * 127, -arm->maxSpeed, arm->maxSpeed) / 127;
            printf("%f %f\n", speed * 127, arm->error);
            if (abs(arm->error) > 0.5)
            {
                arm->moveAtSpeed(speed * 127);
            }
        }
        pros::delay(20);
    }
}

Arm::Arm(int leftMotorPort, int rightMotorPort, int potPort)
{
    this->armPID = new PID(0.05, 0.001, 0);
    this->leftArmMotor = new pros::Motor(leftMotorPort);
    this->rightArmMotor = new pros::Motor(rightMotorPort);
    this->armPot = new pros::ADIPotentiometer(potPort, pros::E_ADI_POT_EDR);
    // this->armOffset = 155;
    this->armOffset = 11.233211;
    pros::Task ArmTask(armTask, (void *)this);
}

double Arm::getAngle()
{
    return this->armPot->get_angle() - this->armOffset;
}

void Arm::tareAngle()
{
    this->armOffset = this->armPot->get_angle();
    printf("Arm Offset: %f\n", this->armOffset);
    return;
}

double Arm::updatePID(double error)
{
    return this->armPID->calculate(error);
}

void Arm::moveToAngle(double angle, double maxSpeed)
{
    this->asyncMoveToAngle(angle, maxSpeed);
    this->maxSpeed = maxSpeed;
    while (error > 0.5)
    {
        pros::delay(20);
    }
    return;
}

void Arm::moveAtSpeed(double speed)
{
    this->leftArmMotor->move(speed);
    this->rightArmMotor->move(speed);
    return;
}
void Arm::setMode(bool isAuton)
{
    this->isEnabled = isAuton;
    return;
}
void Arm::asyncMoveToAngle(double angle, double maxSpeed)
{
    this->isEnabled = true;
    armPID->clear();
    this->asyncTargetPos = angle;
    this->maxSpeed = maxSpeed;
    return;
}