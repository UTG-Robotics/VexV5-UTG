void odometry(void *odometryArgs);

struct OdometryArgs
{
    int leftEncoderPort;
    int rightEncoderPort;
    int sideEncoderPort;
    double leftWheelDistance;
    double rightWheelDistance;
    double sideWheelDistance;
};