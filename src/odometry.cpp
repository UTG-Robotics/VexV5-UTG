#include "main.h"

void odometry(void *odometryArgs)
{
    ((OdometryArgs *)odometryArgs)->x = 0;
    ((OdometryArgs *)odometryArgs)->y = 0;
    ((OdometryArgs *)odometryArgs)->theta = 0;

    pros::Rotation leftEncoder(((OdometryArgs *)odometryArgs)->leftEncoderPort);
    pros::Rotation rightEncoder(((OdometryArgs *)odometryArgs)->rightEncoderPort);
    pros::Rotation sideEncoder(12);

    long int currentLeft = 0;
    long int currentRight = 0;
    long int oldLeft = 0;
    long int oldRight = 0;

    long int deltaLeft = 0;
    long int deltaRight = 0;
    double deltaTheta = 0;

    while (true)
    {
        oldLeft = currentLeft;
        oldRight = currentRight;

        // Get the current encoder values
        currentLeft = leftEncoder.get_position();
        currentRight = rightEncoder.get_position();

        deltaLeft = currentLeft - oldLeft;
        deltaRight = currentRight - oldRight;

        // Calculate the delta theta traveled
        deltaTheta = (deltaLeft - deltaRight) / (((OdometryArgs *)odometryArgs)->leftWheelDistance + ((OdometryArgs *)odometryArgs)->rightWheelDistance);
        //     *((OdometryArgs *)odometryArgs)->theta += deltaTheta;
        pros::lcd::set_text(7, std::to_string(deltaTheta));
        pros::delay(10);
    }
}