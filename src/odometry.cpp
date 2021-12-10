#include "main.h"

double modulo(double a, double b)
{
    while (a > b)
    {
        a -= b;
    }
    return a;
}

void odometry(void *odometryArgs)
{
    xPos = 0;
    yPos = 0;
    angle = 0;

    pros::Rotation leftEncoder(((OdometryArgs *)odometryArgs)->leftEncoderPort);
    pros::Rotation rightEncoder(((OdometryArgs *)odometryArgs)->rightEncoderPort);
    pros::Rotation sideEncoder(((OdometryArgs *)odometryArgs)->sideEncoderPort);

    // pros::Rotation sideEncoder(12);to_string

    // pros::lcd::print(7, "%p", (((OdometryArgs *)odometryArgs)->theta));

    const double ENCODERTOINCHES = 0.00023998277;
    double leftReset = 0;
    double rightReset = 0;
    double thetaReset = 0;

    double currentLeftEncoder = leftEncoder.get_position();
    double currentRightEncoder = rightEncoder.get_position();
    double currentSideEncoder = sideEncoder.get_position();

    double oldLeftEncoder = 0;
    double oldRightEncoder = 0;
    double oldSideEncoder = 0;

    double deltaLeft = 0;
    double deltaRight = 0;
    double deltaSide = 0;
    double deltaTheta = 0;

    double theta = 0;
    double thetaNew = 0;

    leftEncoder.reset();
    rightEncoder.reset();
    sideEncoder.reset();
    leftEncoder.reset_position();
    rightEncoder.reset_position();
    sideEncoder.reset_position();
    while (true)
    {
        // Get the current encoder values in degrees
        currentLeftEncoder = leftEncoder.get_position() * ENCODERTOINCHES;
        currentRightEncoder = rightEncoder.get_position() * ENCODERTOINCHES;
        currentSideEncoder = sideEncoder.get_position() * ENCODERTOINCHES;

        deltaLeft = -(currentLeftEncoder - oldLeftEncoder);
        deltaRight = (currentRightEncoder - oldRightEncoder);
        deltaSide = (currentSideEncoder - oldSideEncoder);

        oldLeftEncoder = currentLeftEncoder;
        oldRightEncoder = currentRightEncoder;
        oldSideEncoder = currentSideEncoder;

        // double deltaLeftReset = (currentLeftEncoder - leftReset) * ENCODERTOINCHES;
        // double deltaRightReset = (currentRightEncoder - rightReset) * ENCODERTOINCHES;

        // deltaLeft = 1;
        // deltaRight = -1;
        // theta = 0;

        // Calculate the change in angle
        thetaNew = (deltaLeft - deltaRight) / (((OdometryArgs *)odometryArgs)->leftWheelDistance + ((OdometryArgs *)odometryArgs)->rightWheelDistance);

        double i;
        double h;
        double h2;
        if (thetaNew)
        {

            double r = deltaRight / thetaNew; // The radius of the circle the robot travel's around with the right side of the robot
            i = thetaNew / 2.0;
            double sinI = sin(i);
            h = ((r + ((OdometryArgs *)odometryArgs)->rightWheelDistance) * sinI) * 2.0;

            double r2 = deltaSide / thetaNew; // The radius of the circle the robot travel's around with the back of the robot
            h2 = ((r2 + ((OdometryArgs *)odometryArgs)->sideWheelDistance) * sinI) * 2.0;

            // h = ((deltaRight / thetaNew + ((OdometryArgs *)odometryArgs)->rightWheelDistance) * sin(thetaNew / 2.0)) * 2.0;

            // h2 = ((deltaSide / thetaNew + ((OdometryArgs *)odometryArgs)->sideWheelDistance) * sin(thetaNew / 2.0)) * 2.0;
        }
        else
        {
            h = deltaRight;
            i = 0;
            h2 = deltaSide;
        }

        double p = i + angle;

        double cosP = cos(p);
        double sinP = sin(p);

        yPos += h * cosP;
        xPos += h * sinP;

        yPos += h2 * -sinP; // -sin(x) = sin(-x)
        xPos += h2 * cosP;  // cos(x) = cos(-x)

        angle += thetaNew;

        pros::lcd::set_text(1, "h: " + std::to_string(deltaTheta));

        pros::lcd::set_text(4, "L: " + std::to_string(oldLeftEncoder) + " R: " + std::to_string(oldRightEncoder));
        pros::lcd::set_text(5, "Rotation: " + std::to_string(angle * 180 / M_PI));
        pros::lcd::set_text(6, "X pos: " + std::to_string(xPos));
        pros::lcd::set_text(7, "Y pos: " + std::to_string(yPos));
        pros::delay(10);
    }
}
