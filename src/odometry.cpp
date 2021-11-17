#include "main.h"

void odometry(void *odometryArgs)
{
    xPos = 0;
    yPos = 0;
    theta = 0;

    pros::Rotation leftEncoder(((OdometryArgs *)odometryArgs)->leftEncoderPort);
    pros::Rotation rightEncoder(((OdometryArgs *)odometryArgs)->rightEncoderPort);

    // pros::Rotation sideEncoder(12);to_string

    // pros::lcd::print(7, "%p", (((OdometryArgs *)odometryArgs)->theta));

    double deltaLeft = 0;
    double deltaRight = 0;

    double deltaTheta = 0;

    leftEncoder.reset();
    rightEncoder.reset();
    leftEncoder.reset_position();
    rightEncoder.reset_position();
    while (true)
    {
        // Get the current encoder values in degrees
        deltaLeft = -(leftEncoder.get_position() / 100.0);
        deltaRight = (rightEncoder.get_position() / 100.0);

        // multiply by wheel circumference then divide by 360 degrees to get inches
        deltaLeft *= (2.75 * M_PI) / 360.0;
        deltaRight *= (2.75 * M_PI) / 360.0;

        // Calculate the change in angle
        deltaTheta = (deltaLeft - deltaRight) / (((OdometryArgs *)odometryArgs)->leftWheelDistance + ((OdometryArgs *)odometryArgs)->rightWheelDistance);

        //Accumulate the change in angle
        theta += deltaTheta * 180 / M_PI;

        leftEncoder.reset_position();
        rightEncoder.reset_position();
        pros::lcd::set_text(7, std::to_string(theta));
        pros::delay(10);
    }
}