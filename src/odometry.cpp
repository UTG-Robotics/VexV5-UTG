#include "main.h"

float Sl = 6.69;            //distance from tracking center to middle of left wheel
float Sr = 6.69;            //distance from tracking center to middle of right wheel
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

pros::Rotation leftEncoder(11);
pros::Rotation rightEncoder(20);
pros::Rotation sideEncoder(7);

pros::IMU gyro(3);

void updatePosition(int i)
{
    //https://gm0.org/en/latest/docs/software/odometry.html How to odometry

    curLeft = leftEncoder.get_position() / 100;
    curRight = rightEncoder.get_position() / 100; //step 1
    curSide = sideEncoder.get_position() / 100;

    deltaLeft = (curLeft - lastLeftPos) * (M_PI / 180) * (wheelDiameter / 2);
    deltaRight = (curRight - lastRightPos) * (M_PI / 180) * (wheelDiameter / 2); //step 2
    deltaSide = (curSide - lastSidePos) * (M_PI / 180) * (wheelDiameter / 2);

    //Calculate change in rotation
    deltaTheta = -(deltaLeft - deltaRight) / (Sl + Sr);

    //Calculate change in X and Y position
    delta_middle_pos = (deltaLeft + deltaRight) / 2;
    delta_perp_pos = deltaSide - Ss * deltaTheta;

    deltaX = delta_middle_pos * cos(angle) - delta_perp_pos * sin(angle);
    deltaY = delta_middle_pos * sin(angle) + delta_perp_pos * cos(angle);

    //Acumulate change in X, Y and Rotation
    // xPos += deltaX;
    // yPos += deltaY;
    xPos += deltaY;
    yPos += deltaX;
    angle += deltaTheta;
    if (i % 5 == 0)
    {
        // printf("%f %f %f %f %f %f %f %f %f %f\n", deltaLeft, deltaRight, deltaTheta, curLeft, curRight, angle, deltaSide, curSide, xPos, yPos);
    }
    pros::lcd::set_text(2, "X: " + std::to_string(xPos) + " Y: " + std::to_string(yPos));
    pros::lcd::set_text(3, "Left: " + std::to_string(curLeft) + " Right: " + std::to_string(curRight));
    pros::lcd::set_text(4, "Angle: " + std::to_string(angle * 180 / M_PI));
    pros::lcd::set_text(5, "Gyro Angle: " + std::to_string(gyro.get_rotation()));

    //Store old encoder values
    lastLeftPos = curLeft;
    lastRightPos = curRight;
    lastSidePos = curSide;
}
void odometry(void *odometryArgs)
{
    xPos = 0;
    yPos = 0;
    angle = 0;

    leftEncoder.set_reversed(false);
    rightEncoder.set_reversed(true);
    sideEncoder.set_reversed(false);
    int i = 0;
    while (true)
    {
        // if (i % 10 == 0)
        // {
        //     printf("%f %f %f %f %f %f\n", deltaLeft, deltaRight, deltaTheta, curLeft, curRight, angle);
        // }
        updatePosition(i);
        pros::delay(20);
        i++;
    }
}
