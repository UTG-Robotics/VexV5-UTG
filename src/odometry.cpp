#include "main.h"

void odometry(void *odometryArgs)
{
    xPos = 0;
    yPos = 0;
    theta = 0;

    pros::Rotation leftEncoder(((OdometryArgs *)odometryArgs)->leftEncoderPort);
    pros::Rotation rightEncoder(((OdometryArgs *)odometryArgs)->rightEncoderPort);
    pros::Rotation sideEncoder(((OdometryArgs *)odometryArgs)->sideEncoderPort);

    // pros::Rotation sideEncoder(12);to_string

    // pros::lcd::print(7, "%p", (((OdometryArgs *)odometryArgs)->theta));

    double deltaLeft = 0;
    double deltaRight = 0;
    double deltaSide = 0;
    double deltaTheta = 0;

    leftEncoder.reset();
    rightEncoder.reset();
    sideEncoder.reset();
    leftEncoder.reset_position();
    rightEncoder.reset_position();
    sideEncoder.reset_position();
    while (true)
    {
        // Get the current encoder values in degrees
        deltaLeft = -(leftEncoder.get_position() / 100.0);
        deltaRight = (rightEncoder.get_position() / 100.0);
        deltaSide = (sideEncoder.get_position() / 100.0);

        // multiply by wheel circumference then divide by 360 degrees to get inches
        deltaLeft *= (2.75 * M_PI) / 360.0;
        deltaRight *= (2.75 * M_PI) / 360.0;
        deltaSide *= (2.75 * M_PI) / 360.0;

        // deltaLeft = 1;
        // deltaRight = -1;
        // theta = 0;

        // Calculate the change in angle
        deltaTheta = (deltaLeft - deltaRight) / (((OdometryArgs *)odometryArgs)->leftWheelDistance + ((OdometryArgs *)odometryArgs)->rightWheelDistance);

        //Accumulate the change in angle
        theta += deltaTheta * 180 / M_PI;
        double thetaRad = theta * M_PI / 180;

        double h;
        double h2;
        double i;
        if (deltaTheta)
        {
            double radius = deltaRight / deltaTheta;
            i = deltaTheta / 2.0;
            double sinI = sin(i);

            h = ((radius + ((OdometryArgs *)odometryArgs)->rightWheelDistance) * sinI) * 2.0;

            double radius2 = deltaSide / deltaTheta;

            h2 = ((radius2 + ((OdometryArgs *)odometryArgs)->sideWheelDistance) * sinI) * 2.0;
        }
        else
        {
            h = deltaRight;
            i = 0;
            h2 = deltaSide;
        }
        double p = i + thetaRad;

        double cosP = cos(p);
        double sinP = sin(p);

        yPos += h * cosP;
        xPos += h * sinP;

        yPos += h2 * -sinP;
        xPos += h2 * cosP;

        leftEncoder.reset_position();
        rightEncoder.reset_position();
        sideEncoder.reset_position();

        pros::lcd::set_text(5, "Rotation: " + std::to_string(theta));
        pros::lcd::set_text(6, "X pos: " + std::to_string(xPos));
        pros::lcd::set_text(7, "Y pos: " + std::to_string(yPos));
        pros::delay(10);
    }
}