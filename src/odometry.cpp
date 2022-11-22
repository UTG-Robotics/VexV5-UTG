#include "main.h"
// old = 6.55
double Sl = 6.77, Sr = Sl; // distance from tracking center to middle of left and right wheel

double Ss = 6.56; // distance from tracking center to middle of the tracking wheel
// double Ss = 1.56;            // distance from tracking center to middle of the tracking wheel
double wheelDiameter = 2.75; // diameter of the wheels being used for tracking

double lastLeftPos = 0;
double lastRightPos = 0;
double lastSidePos = 0;
double oldGyro = 0;

double curLeft = 0;
double curRight = 0;
double curSide = 0;
double curGyro = 0;

double deltaLeft = 0;
double deltaRight = 0;
double deltaSide = 0;
double deltaTheta = 0;

double localX = 0;
double localY = 0;

double globalX = 0;
double globalY = 0;

double delta_forward_pos = 0;
double delta_side_pos = 0;

double deltaX = 0;
double deltaY = 0;

double posX = 0;
double posY = 0;

// pros::Rotation leftEncoder(11);
// pros::Rotation rightEncoder(20);
// pros::Rotation sideEncoder(7);

// pros::IMU gyro(3);

pros::IMU gyro(4);
// pros::Rotation leftEncoder(12);
// pros::Rotation rightEncoder(3);
// pros::Rotation sideEncoder(18);

pros::Rotation leftEncoder(12);
pros::Rotation rightEncoder(1);
pros::Rotation sideEncoder(18);

void updatePosition()
{
    // Save the current encoder/gyro values
    curLeft = leftEncoder.get_position() / 100;
    curRight = rightEncoder.get_position() / 100;
    curSide = sideEncoder.get_position() / 100;
    curGyro = gyro.get_rotation() * M_PI / 180;
    // printf("%f,%f,%f,%f\n", curLeft, curRight, curSide, curGyro);

    // Calculate the change in encoder values in inches
    deltaLeft = (curLeft - lastLeftPos) * (M_PI / 180) * (wheelDiameter / 2);
    deltaRight = (curRight - lastRightPos) * (M_PI / 180) * (wheelDiameter / 2);
    deltaSide = (curSide - lastSidePos) * (M_PI / 180) * (wheelDiameter / 2);

    // Calculate change in rotation by averaging encoders and gyro
    // deltaTheta = ((curGyro - oldGyro) * 0.67 + (deltaLeft - deltaRight) / (Sl + Sr) * 1.33) / 2;

    // deltaTheta = ((deltaLeft - deltaRight) / (Sl + Sr));
    // angle += deltaTheta;

    angle = curGyro;
    deltaTheta = -(angle - oldGyro);

    // Store old encoder/gyro values
    lastLeftPos = curLeft;
    lastRightPos = curRight;
    lastSidePos = curSide;
    oldGyro = curGyro;

    // Calculate change in forward and sideways position
    if (deltaTheta)
    {
        localX = (deltaTheta + (deltaSide / Ss)) * Ss;
        localY = (deltaLeft + deltaRight) / 2;

        globalX = localY * sin(angle - deltaTheta / 2) + localX * cos(angle - deltaTheta / 2);
        globalY = localY * cos(angle - deltaTheta / 2) - localX * sin(angle - deltaTheta / 2);

        // Acumulate change in X, Y and Rotation
        posX += globalX;
        posY += globalY;
    }
    else
    {
        posX += deltaSide;
        posY += deltaRight;
        // localX = deltaRight;
        // localY = deltaSide;
    }
    // double p = angle - deltaTheta / 2.0; // global angle

    // convert to absolute displacement
    // xPos += cos(p) * localX - sin(p) * localY;
    // yPos += cos(p) * localY + sin(p) * localX;
    /*
    ensure all numbers are real numbers
    in case of invalid sensor data
    */
    if (deltaTheta != deltaTheta)
        deltaTheta = 0;
    if (angle != angle)
        angle = 0;
    if (xPos != xPos)
        xPos = 0;
    if (yPos != yPos)
        yPos = 0;

    printf("\n%f,%f,%f", posX, posY, angle);
    // Debug Info

    pros::lcd::set_text(2, "X: " + std::to_string(xPos) + " Y: " + std::to_string(yPos));
    pros::lcd::set_text(3, "Left: " + std::to_string(curLeft) + " Right: " + std::to_string(curRight));
    pros::lcd::set_text(4, "Angle: " + std::to_string(angle * 180 / M_PI));
    pros::lcd::set_text(5, "Gyro Angle: " + std::to_string(gyro.get_rotation()));
    pros::lcd::set_text(6, "Old Gyro Angle: " + std::to_string(gyro.get_rotation() - oldGyro));
    pros::lcd::set_text(7, "Wheel Distance: " + std::to_string(angle * (Sr + Sl) / (1800 * M_PI / 180)));
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
    sideEncoder.set_reversed(false);

    leftEncoder.reset();
    rightEncoder.reset();
    sideEncoder.reset();
    leftEncoder.reset_position();
    rightEncoder.reset_position();
    sideEncoder.reset_position();

    int i = 0;
    // printf("X,Y,Angle");
    pros::delay(200);
    while (true)
    {
        // wait for gyro initialization
        if (gyro.get_rotation() != gyro.get_rotation() || std::isinf(gyro.get_rotation()))
        {
            // printf("Gyro not initialized\n");
            continue;
        }

        if (i > 10)
        {
            updatePosition();
        }
        else
        {
            i++;
            xPos = 0;
            yPos = 0;
            angle = 0;
        }
        pros::delay(10);
    }
}
