#include "main.h"

double modulo(double a, double b)
{
    while (a > b)
    {
        a -= b;
    }
    return a;
}
float Sl = 6.27;               //distance from tracking center to middle of left wheel
float Sr = 6.27;               //distance from tracking center to middle of right wheel
float Ss = 100;                //distance from tracking center to middle of the tracking wheel
float wheelDiameter = 2.75;    //diameter of the side wheels being used for tracking
float trackingDiameter = 2.75; //diameter of the sideways tracking wheel

// double xPos = 0;
// double yPos = 0;
// double angle = 0;

float lastLeftPos = 0;
float lastRightPos = 0;
float lastSidePos = 0;

float deltaTheta = 0;
float thetaNew = 0;
float thetaM = 0;

float curLeft = 0;
float curRight = 0;
float curSide = 0;

float leftAtReset = 0;
float rightAtReset = 0;
float thetaReset = 0;

float deltaLeft = 0;
float deltaRight = 0;
float deltaSide = 0;

float deltaLr = 0;
float deltaRr = 0;

float deltaX = 0;
float deltaY = 0;

float theta = 0;
float radius = 0;

uint32_t lastTime = pros::millis();

pros::Rotation leftEncoder(12);
pros::Rotation rightEncoder(3);
pros::Rotation sideEncoder(19);

void updatePosition()
{
    curLeft = -leftEncoder.get_position() / 100;
    curRight = rightEncoder.get_position() / 100; //step 1
    curSide = sideEncoder.get_position() / 100;

    deltaLeft = (curLeft - lastLeftPos) * (M_PI / 180) * (wheelDiameter / 2);
    deltaRight = (curRight - lastRightPos) * (M_PI / 180) * (wheelDiameter / 2); //step 2
    deltaSide = (curSide - lastSidePos) * (M_PI / 180) * (trackingDiameter / 2);

    lastLeftPos = curLeft;

    deltaLr = (curLeft - leftAtReset) * (M_PI / 180) * (wheelDiameter / 2); //step 4
    deltaRr = (curRight - rightAtReset) * (M_PI / 180) * (wheelDiameter / 2);

    thetaNew = (thetaReset + (deltaLr - deltaRr) / (Sl + Sr)); //step 5
    // if (thetaNew >= M_PI) {
    //   thetaNew-=2*M_PI;
    // }
    // else if (thetaNew <= -M_PI) {
    //   thetaNew+=2*M_PI;
    // }

    deltaTheta = thetaNew - angle; //step 6

    // deltaSide = deltaSide - Ss * deltaTheta;

    if (deltaTheta == 0)
    {
        deltaX = deltaSide; //step 7
        deltaY = deltaRight;
    }
    else
    {
        deltaX = (2 * sin(deltaTheta / 2)) * (deltaSide / deltaTheta + Ss); //step 8
        deltaY = (2 * sin(deltaTheta / 2)) * (deltaRight / deltaTheta + Sr);
    }

    // thetaM = angle + deltaTheta / 2; //step 9

    // theta = atan2(deltaY, deltaX);
    // radius = sqrt(deltaX * deltaX + deltaY * deltaY);
    // theta = theta - thetaM; //step 10
    // deltaX = radius * cos(theta);
    // deltaY = radius * sin(theta);

    // xPos = xPos - deltaX; //step 11
    // yPos = yPos + deltaY;

    float p = (thetaNew / 2) + angle; // The global ending angle of the robot

    float cosP = cos(p);
    float sinP = sin(p);

    yPos += deltaY * cosP;
    xPos += deltaY * sinP;

    yPos += deltaX * -sinP; // -sin(x) = sin(-x)
    xPos += deltaX * cosP;  // cos(x) = cos(-x)

    angle = thetaNew;
    printf("x: %f, y: %f, theta: %f, deltaX: %f, deltaY: %f    time:%i\n", xPos, yPos, angle, deltaX, deltaY, ((uint32_t)pros::millis() - (uint32_t)lastTime));
    lastLeftPos = curLeft;
    lastRightPos = curRight;
    lastSidePos = curSide;

    lastTime = pros::millis();
}
void odometry(void *odometryArgs)
{
    //     while (true)
    //     {
    //         updatePosition();
    //         pros::Task::delay(20);
    //     }
    // }
    xPos = 0;
    yPos = 0;
    angle = 0;
    // pros::Motor front_right_mtr(10);
    // pros::Motor front_left_mtr(1);
    // pros::Motor back_right_mtr(20);
    // pros::Motor back_left_mtr(11);

    okapi::Motor frontLeftMotor(1, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
    okapi::Motor frontRightMotor(10, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
    okapi::Motor backRightMotor(20, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
    okapi::Motor backLeftMotor(11, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);

    pros::Rotation leftEncoder(12);
    leftEncoder.set_reversed(true);
    pros::Rotation rightEncoder(3);
    pros::Rotation sideEncoder(19);

    std::shared_ptr<okapi::OdomChassisController> chassis = okapi::ChassisControllerBuilder()
                                                                .withMotors(frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor)
                                                                .withSensors(okapi::RotationSensor(12, true), okapi::RotationSensor(3), okapi::RotationSensor(19))
                                                                .withDimensions(okapi::AbstractMotor::gearset::green, {{4_in, 11.5_in}, okapi::imev5GreenTPR}) //gearset, diameter, track, ticks
                                                                .withOdometry({{2.75_in, 7_in, 6_in, 2.75_in}, 360})
                                                                .buildOdometry(); // build an odometry chassis

    while (true)
    {
        // updatePosition();
        printf("x: %f\n", chassis->getState().x.convert(okapi::inch));
        pros::delay(10);
    }
}
