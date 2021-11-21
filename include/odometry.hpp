void odometry(void *odometryArgs);

struct OdometryArgs
{
    int leftEncoderPort;
    int rightEncoderPort;
    double leftWheelDistance;
    double rightWheelDistance;
};