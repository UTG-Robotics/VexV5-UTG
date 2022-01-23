#include "main.h"

float Sl = 6.27;            //distance from tracking center to middle of left wheel
float Sr = 6.27;            //distance from tracking center to middle of right wheel
float Ss = 5.5;             //distance from tracking center to middle of the tracking wheel
float wheelDiameter = 2.75; //diameter of the side wheels being used for tracking

float lastLeftPos = 0;
float lastRightPos = 0;
float lastSidePos = 0;

float curLeft = 0;
float curRight = 0;
float curSide = 0;

float deltaLeft = 0;
float deltaRight = 0;
float deltaSide = 0;
float deltaTheta = 0;

float delta_middle_pos = 0;
float delta_perp_pos = 0;

float deltaX = 0;
float deltaY = 0;

pros::Rotation leftEncoder(12);
pros::Rotation rightEncoder(3);
pros::Rotation sideEncoder(19);

void updatePosition()
{
    //https://gm0.org/en/latest/docs/software/odometry.html How to odometry

    //Get amount of inches traveled for each encoder
    curLeft = leftEncoder.get_position() / 100 * (M_PI / 180) * (wheelDiameter / 2);
    curRight = rightEncoder.get_position() / 100 * (M_PI / 180) * (wheelDiameter / 2);
    curSide = sideEncoder.get_position() / 100 * (M_PI / 180) * (wheelDiameter / 2);

    //Calculate change in encoder values since last calculation
    deltaLeft = curLeft - lastLeftPos;
    deltaRight = curRight - lastRightPos;
    deltaSide = curSide - lastSidePos;

    //Store old encoder values
    lastLeftPos = curLeft;
    lastRightPos = curRight;
    lastSidePos = curSide;

    //Calculate change in rotation
    deltaTheta = -(deltaLeft - deltaRight) / (Sl + Sr);

    //Canculate change in X and Y position
    delta_middle_pos = (deltaLeft + deltaRight) / 2;
    delta_perp_pos = deltaSide - Ss * deltaTheta;

    deltaX = delta_middle_pos * cos(angle) - delta_perp_pos * sin(angle);
    deltaY = delta_middle_pos * sin(angle) + delta_perp_pos * cos(angle);

    //Acumulate change in X, Y and Rotation
    xPos += deltaX;
    yPos += deltaY;
    angle += deltaTheta;

    pros::lcd::set_text(2, "X: " + std::to_string(xPos) + " Y: " + std::to_string(yPos));
    pros::lcd::set_text(3, "Angle: " + std::to_string(angle * 180 / M_PI));
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
        pros::delay(10);
    }
}
