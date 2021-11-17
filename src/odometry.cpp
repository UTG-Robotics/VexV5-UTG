#include "main.h"

void odometry(void *odometryArgs)
{
    *((OdometryArgs *)odometryArgs)->x = 0;
    *((OdometryArgs *)odometryArgs)->y = 0;
    *((OdometryArgs *)odometryArgs)->theta = 0;

    pros::Rotation leftEncoder(((OdometryArgs *)odometryArgs)->leftEncoderPort);
    pros::Rotation rightEncoder(((OdometryArgs *)odometryArgs)->rightEncoderPort);

    // pros::Rotation sideEncoder(12);to_string

    // pros::lcd::print(7, "%p", (((OdometryArgs *)odometryArgs)->theta));

    double currentLeft = 0;
    double currentRight = 0;
    double oldLeft = 0;
    double oldRight = 0;

    double deltaLeft = 0;
    double deltaRight = 0;
    double deltaTheta = 0;

    leftEncoder.reset();
    rightEncoder.reset();
    leftEncoder.reset_position();
    rightEncoder.reset_position();
    while (true)
    {
        // oldLeft = currentLeft;
        // oldRight = currentRight;

        // Get the current encoder values in degrees
        currentLeft = -(leftEncoder.get_position() / 100.0);
        currentRight = (rightEncoder.get_position() / 100.0);

        // multiply by wheel diameter then divide by 360 degrees to get inches
        currentLeft *= (2.75 * M_PI) / 360.0;
        currentRight *= (2.75 * M_PI) / 360.0;

        // Calculate the change in distance
        // deltaLeft = (currentLeft - oldLeft);
        // deltaRight = (currentRight - oldRight);

        // Calculate the change in angle
        deltaTheta = (currentLeft - currentRight) / (((OdometryArgs *)odometryArgs)->leftWheelDistance + ((OdometryArgs *)odometryArgs)->rightWheelDistance);

        //Accumulate the change in angle
        *(((OdometryArgs *)odometryArgs)->theta) += (double)deltaTheta;

        leftEncoder.reset_position();
        rightEncoder.reset_position();
        pros::delay(10);
    }
}