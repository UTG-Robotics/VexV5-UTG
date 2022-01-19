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
float Ss = 5;                  //distance from tracking center to middle of the tracking wheel
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

float delta_middle_pos = 0;
float delta_perp_pos = 0;

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
    curLeft = leftEncoder.get_position() / 100;
    curRight = rightEncoder.get_position() / 100; //step 1
    curSide = sideEncoder.get_position() / 100;

    deltaLeft = (curLeft - lastLeftPos) * (M_PI / 180) * (wheelDiameter / 2);
    deltaRight = (curRight - lastRightPos) * (M_PI / 180) * (wheelDiameter / 2); //step 2
    deltaSide = (curSide - lastSidePos) * (M_PI / 180) * (trackingDiameter / 2);

    deltaTheta = (deltaLeft - deltaRight) / (Sl + Sr); //step 5

    delta_middle_pos = (deltaLeft + deltaRight) / 2;
    delta_perp_pos = deltaSide - Ss * deltaTheta;

    deltaX = delta_middle_pos * cos(angle) - delta_perp_pos * sin(angle);
    deltaY = delta_middle_pos * sin(angle) + delta_perp_pos * cos(angle);

    xPos += deltaX;
    yPos += deltaY;
    angle += deltaTheta;
    // printf("x: %f, y: %f, theta: %f, deltaX: %f, deltaY: %f    time:%i\n", xPos, yPos, angle * 180 / M_PI, deltaX, deltaY, ((uint32_t)pros::millis() - (uint32_t)lastTime));
    lastLeftPos = curLeft;
    lastRightPos = curRight;
    lastSidePos = curSide;

    lastTime = pros::millis();
}
void odometry(void *odometryArgs)
{
    xPos = 0;
    yPos = 0;
    angle = 0;

    leftEncoder.set_reversed(true);
    sideEncoder.set_reversed(true);

    while (true)
    {
        updatePosition();
        // printf("x: %f\n", chassis->getState().x.convert(okapi::inch));
        pros::delay(10);

        //https://gm0.org/en/latest/docs/software/odometry.html How to odometry
    }
}
