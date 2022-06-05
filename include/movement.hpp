void rotateToAngle(double angle);
void driveForward(double inches);
void driveToPoint(double x, double y, double angle, double maxSpeed = 60, int timeOut = 100000);
void move_relative_blocking(pros::Motor &targetMotor, int amount, int rpm, int timeOut = 10000);
double slew(double target, double current, double slewRate);
extern double targetX;
extern double targetY;
extern double targetAngleGlobal;