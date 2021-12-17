#include "main.h"

double modulo(double a, double b)
{
    while (a > b)
    {
        a -= b;
    }
    return a;
}
float Sl = 6.27;               //distance from tracking center to middle of left wheel
float Sr = 6.27;               //distance from tracking center to middle of right wheel
float Ss = 100;                //distance from tracking center to middle of the tracking wheel
float wheelDiameter = 2.75;    //diameter of the side wheels being used for tracking
float trackingDiameter = 2.75; //diameter of the sideways tracking wheel

// double xPos = 0;
// double yPos = 0;
// double angle = 0;

float lastLeftPos = 0;
float lastRightPos = 0;
float lastSidePos = 0;

float deltaTheta = 0;
float thetaNew = 0;
float thetaM = 0;

float curLeft = 0;
float curRight = 0;
float curSide = 0;

float leftAtReset = 0;
float rightAtReset = 0;
float thetaReset = 0;

float deltaLeft = 0;
float deltaRight = 0;
float deltaSide = 0;

float deltaLr = 0;
float deltaRr = 0;

float deltaX = 0;
float deltaY = 0;

float theta = 0;
float radius = 0;

uint32_t lastTime = pros::millis();

pros::Rotation leftEncoder(12);
pros::Rotation rightEncoder(3);
pros::Rotation sideEncoder(19);

void updatePosition()
{
    curLeft = -leftEncoder.get_position() / 100;
    curRight = rightEncoder.get_position() / 100; //step 1
    curSide = sideEncoder.get_position() / 100;

    deltaLeft = (curLeft - lastLeftPos) * (M_PI / 180) * (wheelDiameter / 2);
    deltaRight = (curRight - lastRightPos) * (M_PI / 180) * (wheelDiameter / 2); //step 2
    deltaSide = (curSide - lastSidePos) * (M_PI / 180) * (trackingDiameter / 2);

    lastLeftPos = curLeft;
    lastRightPos = curRight; //step 3
    lastSidePos = curSide;

    deltaLr = (curLeft - leftAtReset) * (M_PI / 180) * (wheelDiameter / 2); //step 4
    deltaRr = (curRight - rightAtReset) * (M_PI / 180) * (wheelDiameter / 2);

    thetaNew = (thetaReset + (deltaLr - deltaRr) / (Sl + Sr)); //step 5
    // if (thetaNew >= M_PI) {
    //   thetaNew-=2*M_PI;
    // }
    // else if (thetaNew <= -M_PI) {
    //   thetaNew+=2*M_PI;
    // }

    deltaTheta = thetaNew - angle; //step 6

    deltaSide = deltaSide - Ss * deltaTheta;

    if (deltaTheta == 0)
    {
        deltaX = deltaSide; //step 7
        deltaY = deltaRight;
    }
    else
    {
        deltaX = (2 * sin(deltaTheta / 2)) * (deltaSide / deltaTheta + Ss); //step 8
        deltaY = (2 * sin(deltaTheta / 2)) * (deltaRight / deltaTheta + Sr);
    }

    thetaM = angle + deltaTheta / 2; //step 9

    // if (deltaX == 0) {
    //   if (deltaY > 0) {
    //     theta = M_PI/2;
    //   }
    //   else if (deltaY < 0) {
    //     theta = 3*M_PI/2;
    //   }
    //   else {
    //     theta = 0;
    //   }
    // }
    // else {
    //   theta = atan(deltaY/deltaX);
    // }
    theta = atan2(deltaY, deltaX);
    radius = sqrt(deltaX * deltaX + deltaY * deltaY);
    theta = theta - thetaM; //step 10
    deltaX = radius * cos(theta);
    deltaY = radius * sin(theta);

    thetaNew += M_PI;
    while (thetaNew <= 0)
    {
        thetaNew += 2 * M_PI;
    }
    thetaNew = modulo(thetaNew, 2 * M_PI);
    thetaNew -= M_PI;

    angle = thetaNew;
    xPos = xPos - deltaX * (10 / ((uint32_t)pros::millis() - (uint32_t)lastTime)); //step 11
    yPos = yPos + deltaY;
    printf("x: %f, y: %f, theta: %f, deltaX: %f, deltaY: %f    time:%i\n", xPos, yPos, angle, deltaX, deltaY, ((uint32_t)pros::millis() - (uint32_t)lastTime));
    lastTime = pros::millis();
}
void odometry(void *odometryArgs)
{
    //     while (true)
    //     {
    //         updatePosition();
    //         pros::Task::delay(20);
    //     }
    // }
    xPos = 0;
    yPos = 0;
    angle = 0;

    // pros::Rotation leftEncoder(((OdometryArgs *)odometryArgs)->leftEncoderPort);
    // pros::Rotation rightEncoder(((OdometryArgs *)odometryArgs)->rightEncoderPort);
    // pros::Rotation sideEncoder(((OdometryArgs *)odometryArgs)->sideEncoderPort);

    // // pros::Rotation sideEncoder(12);to_string

    // // pros::lcd::print(7, "%p", (((OdometryArgs *)odometryArgs)->theta));

    // const double ENCODERTOINCHES = 0.00023998277;
    // double leftReset = 0;
    // double rightReset = 0;
    // double thetaReset = 0;

    // double currentLeftEncoder = leftEncoder.get_position();
    // double currentRightEncoder = rightEncoder.get_position();
    // double currentSideEncoder = sideEncoder.get_position();

    // double oldLeftEncoder = 0;
    // double oldRightEncoder = 0;
    // double oldSideEncoder = 0;

    // double deltaLeft = 0;
    // double deltaRight = 0;
    // double deltaSide = 0;
    // double deltaTheta = 0;

    // double theta = 0;
    // double thetaNew = 0;

    // double radius = 0;

    // leftEncoder.reset();
    // rightEncoder.reset();
    // sideEncoder.reset();
    // leftEncoder.reset_position();
    // rightEncoder.reset_position();
    // sideEncoder.reset_position();
    // while (true)
    // {
    //     // Get the current encoder values in degrees
    //     currentLeftEncoder = -leftEncoder.get_position() * ENCODERTOINCHES;
    //     currentRightEncoder = rightEncoder.get_position() * ENCODERTOINCHES;
    //     currentSideEncoder = sideEncoder.get_position() * ENCODERTOINCHES;

    //     deltaLeft = (currentLeftEncoder - oldLeftEncoder);
    //     deltaRight = (currentRightEncoder - oldRightEncoder);
    //     deltaSide = (currentSideEncoder - oldSideEncoder);

    //     oldLeftEncoder = currentLeftEncoder;
    //     oldRightEncoder = currentRightEncoder;
    //     oldSideEncoder = currentSideEncoder;

    //     // double deltaLeftReset = (currentLeftEncoder - leftReset) * ENCODERTOINCHES;
    //     // double deltaRightReset = (currentRightEncoder - rightReset) * ENCODERTOINCHES;

    //     // deltaLeft = 1;
    //     // deltaRight = -1;
    //     // theta = 0;

    //     // Calculate the change in angle
    //     deltaTheta = (deltaLeft - deltaRight) / (((OdometryArgs *)odometryArgs)->leftWheelDistance + ((OdometryArgs *)odometryArgs)->rightWheelDistance);
    //     // thetaNew = 0;
    //     // deltaTheta = thetaNew - angle;
    //     double i;
    //     double deltaX;
    //     double deltaY;
    //     // deltaSide = deltaSide - ((OdometryArgs *)odometryArgs)->sideWheelDistance * deltaTheta;

    //     if (deltaTheta)
    //     {

    //         i = deltaTheta / 2.0;
    //         double sinI = sin(i);

    //         deltaX = 2.0 * sin(deltaTheta / 2.0) * ((deltaRight / deltaTheta) + ((OdometryArgs *)odometryArgs)->rightWheelDistance);
    //         deltaY = 2.0 * sin(deltaTheta / 2.0) * ((deltaSide / deltaTheta) + ((OdometryArgs *)odometryArgs)->sideWheelDistance);

    //         // deltaX = ((deltaRight / thetaNew + ((OdometryArgs *)odometryArgs)->rightWheelDistance) * sin(thetaNew / 2.0)) * 2.0;

    //         // deltaY = ((deltaSide / thetaNew + ((OdometryArgs *)odometryArgs)->sideWheelDistance) * sin(thetaNew / 2.0)) * 2.0;
    //     }
    //     else
    //     {
    //         deltaX = deltaRight;
    //         i = 0;
    //         deltaY = deltaSide;
    //     }

    //     double thetaM = i + angle;

    //     theta = atan2(deltaY, deltaX);
    //     radius = sqrt(deltaX * deltaX + deltaY * deltaY);
    //     theta -= thetaM; //step 10
    //     deltaX = radius * cos(theta);
    //     deltaY = radius * sin(theta);

    //     // thetaNew += M_PI;
    //     // while (thetaNew <= 0)
    //     // {
    //     //     thetaNew += 2 * M_PI;
    //     // }
    //     // thetaNew = modulo(thetaNew, 2 * M_PI);
    //     // thetaNew -= M_PI;

    //     // double cosP = cos(thetaM);
    //     // double sinP = sin(thetaM);

    //     yPos += deltaY;
    //     xPos += deltaX;

    //     // yPos += deltaY * -sinP; // -sin(x) = sin(-x)
    //     // xPos += deltaY * cosP;  // cos(x) = cos(-x)

    //     angle += deltaTheta;

    //     // xPos = xPos - deltaX; //step 11
    //     // yPos = yPos + deltaY;

    //     pros::lcd::set_text(1, "h: " + std::to_string(radius));
    //     printf("deltaTheta: %f    deltaX: %f    deltaY: %f    Radius: %f\n", deltaTheta, deltaX, deltaY, radius);
    //     pros::lcd::set_text(4, "L: " + std::to_string(oldLeftEncoder) + " R: " + std::to_string(oldRightEncoder));
    //     pros::lcd::set_text(5, "Rotation: " + std::to_string(angle * 180 / M_PI));
    //     pros::lcd::set_text(6, "X pos: " + std::to_string(xPos));
    //     pros::lcd::set_text(7, "Y pos: " + std::to_string(yPos));

    while (true)
    {
        updatePosition();
        pros::delay(10);
    }
}
