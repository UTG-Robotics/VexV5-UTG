void odometry(void *odometryArgs);

struct OdometryArgs
{
    double *x;
    double *y;
    double *theta;
    int leftEncoderPort;
    int rightEncoderPort;
    double leftWheelDistance;
    double rightWheelDistance;
};