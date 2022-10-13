#include "main.h"
// old = 6.55
float Sl = 6.455, Sr = Sl;  // distance from tracking center to middle of left and right wheel
float Ss = 5.4;             // distance from tracking center to middle of the tracking wheel
float wheelDiameter = 2.75; // diameter of the wheels being used for tracking

float lastLeftPos = 0;
float lastRightPos = 0;
float lastSidePos = 0;
float oldGyro = 0;

float curLeft = 0;
float curRight = 0;
float curSide = 0;
float curGyro = 0;

float deltaLeft = 0;
float deltaRight = 0;
float deltaSide = 0;
float deltaTheta = 0;

float delta_forward_pos = 0;
float delta_side_pos = 0;

double deltaLocalX = 0;
double deltaLocalY = 0;
double thetaOffset = 0;
double deltaCoordinateMagnitude = 0;
double deltaCoordinateAngle = 0;

float deltaX = 0;
float deltaY = 0;

pros::IMU gyro(4);
pros::Rotation leftEncoder(3);
pros::Rotation rightEncoder(12);
pros::Rotation sideEncoder(18);

void updatePosition(int i)
{
    // Save the current encoder/gyro values
    curLeft = leftEncoder.get_position() / 100;
    curRight = rightEncoder.get_position() / 100;
    curSide = sideEncoder.get_position() / 100;
    curGyro = gyro.get_rotation() * M_PI / 180;

    printf("left: %f right: %f side: %f ", curLeft * (M_PI / 180) * (wheelDiameter / 2), curRight * (M_PI / 180) * (wheelDiameter / 2), curSide * (M_PI / 180) * (wheelDiameter / 2));
    // Calculate the change in encoder values in inches
    deltaLeft = (curLeft - lastLeftPos) * (M_PI / 180) * (wheelDiameter / 2);
    deltaRight = (curRight - lastRightPos) * (M_PI / 180) * (wheelDiameter / 2);
    deltaSide = (curSide - lastSidePos) * (M_PI / 180) * (wheelDiameter / 2);

    // Calculate change in rotation by averaging encoders and gyro
    deltaTheta = ((deltaLeft - deltaRight) / (Sl + Sr));
    // deltaTheta = (deltaLeft - deltaRight) / (Sl + Sr);

    angle += deltaTheta;

    if (deltaTheta == 0)
    {
        deltaLocalX = deltaSide;
        deltaLocalY = deltaRight;
    }
    else
    {

        deltaLocalX = 2 * sin(angle / 2) * ((deltaSide / deltaTheta) + Ss);
        deltaLocalY = 2 * sin(angle / 2) * ((deltaRight / deltaTheta) + Sr);
    }

    thetaOffset = (angle - deltaTheta) + (deltaTheta / 2);
    // Convert local to polar coordinates
    deltaCoordinateMagnitude = sqrt(deltaLocalX * deltaLocalX + deltaLocalY * deltaLocalY);
    deltaCoordinateAngle = atan2(deltaLocalY, deltaLocalX);

    // Rotate by thetaOffset
    deltaCoordinateAngle -= thetaOffset;

    // Convert back to cartesian coordinates
    deltaX = deltaCoordinateMagnitude * cos(deltaCoordinateAngle);
    deltaY = deltaCoordinateMagnitude * sin(deltaCoordinateAngle);

    // ensure all numbers are real numbers
    if (deltaTheta != deltaTheta)
    {
        deltaTheta = 0;
    }
    if (angle != angle)
    {
        angle = 0;
    }
    if (xPos != xPos)
    {
        xPos = 0;
    }
    if (yPos != yPos)
    {
        yPos = 0;
    }

    // Acumulate change in X, Y and Rotation
    xPos += deltaY;
    yPos += deltaX;

    // Store old encoder/gyro values
    lastLeftPos = curLeft;
    lastRightPos = curRight;
    lastSidePos = curSide;
    oldGyro = curGyro;
    printf("X: %f Y: %f Angle: %f\n", xPos, yPos, angle * 180 / 3.141592653);
    // Debug Info
    /*
    pros::lcd::set_text(2, "X: " + std::to_string(xPos) + " Y: " + std::to_string(yPos));
    pros::lcd::set_text(3, "Left: " + std::to_string(curLeft) + " Right: " + std::to_string(curRight));
    pros::lcd::set_text(4, "Angle: " + std::to_string(angle * 180 / M_PI));
    pros::lcd::set_text(5, "Gyro Angle: " + std::to_string(gyro.get_rotation()));
    pros::lcd::set_text(6, "Old Gyro Angle: " + std::to_string(gyro.get_rotation() - oldGyro));
    pros::lcd::set_text(7, "deltaTheta: " + std::to_string(deltaTheta));
    */
}
void odometry(void *odometryArgs)
{
    // zero position
    xPos = 0;
    yPos = 0;
    angle = 0;

    // configure encoders
    leftEncoder.set_reversed(false);
    rightEncoder.set_reversed(false);
    sideEncoder.set_reversed(true);
    int i = 0;
    pros::delay(200);
    while (true)
    {
        // wait for gyro initialization
        if (gyro.get_rotation() != gyro.get_rotation() || std::isinf(gyro.get_rotation()))
        {
            // printf("Gyro not initialized\n");
            continue;
        }
        updatePosition(i);
        pros::delay(10);
        i++;
    }
}
