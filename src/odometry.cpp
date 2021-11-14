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

    long int currentLeft = 0;
    long int currentRight = 0;
    long int oldLeft = 0;
    long int oldRight = 0;

    long int deltaLeft = 0;
    long int deltaRight = 0;
    double deltaTheta = 0;

    leftEncoder.reset();
    rightEncoder.reset();
    leftEncoder.reset_position();
    rightEncoder.reset_position();
    while (true)
    {
        oldLeft = currentLeft;
        oldRight = currentRight;

        // Get the current encoder values in inches
        currentLeft = -(leftEncoder.get_position() / 100.0);
        currentRight = (rightEncoder.get_position() / 100.0);

        deltaLeft = (currentLeft - oldLeft);
        deltaRight = (currentRight - oldRight);

        // Calculate the delta theta traveled
        deltaTheta = (deltaLeft - deltaRight) / (((OdometryArgs *)odometryArgs)->leftWheelDistance + ((OdometryArgs *)odometryArgs)->rightWheelDistance);
        // pros::lcd::set_text(7, std::to_string(deltaTheta));

        *(((OdometryArgs *)odometryArgs)->theta) += (double)deltaTheta;
        pros::delay(10);
    }
}