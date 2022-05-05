#include "api.h"
class PID;

class Arm
{
private:
    PID *armPID;
    pros::Motor *leftArmMotor;
    pros::Motor *rightArmMotor;
    pros::ADIPotentiometer *armPot;
    double armOffset = 0;

public:
    bool isEnabled = false;
    double error = 0;
    double maxSpeed = 127;
    double asyncTargetPos = 0;
    Arm(int leftMotorPort, int rightMotorPort, int potPort);
    double getAngle();
    void tareAngle();
    void setMode(bool isAuton);
    double updatePID(double error);
    void asyncMoveToAngle(double angle, double maxSpeed);
    void moveToAngle(double angle, double maxSpeed);
    void moveAtSpeed(double speed);
};