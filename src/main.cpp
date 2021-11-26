#include "main.h"

double xPos = 0;
double yPos = 0;
double theta = 0;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	OdometryArgs *odometryArgs = new OdometryArgs();
	odometryArgs->leftEncoderPort = 12;
	odometryArgs->rightEncoderPort = 3;
	odometryArgs->sideEncoderPort = 19;
	//Increasing distance increases overshoot
	odometryArgs->leftWheelDistance = 6.27;
	odometryArgs->rightWheelDistance = 6.27;
	odometryArgs->sideWheelDistance = 5.0;

	pros::Task odometry_task(odometry, odometryArgs);

	pros::lcd::initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol()
{
	// pros::lcd::set_text(7, std::to_string(theta));
	// Initialize the drivetrains
	pros::Controller controller(pros::E_CONTROLLER_MASTER);
	pros::Motor front_right_mtr(10);
	pros::Motor front_left_mtr(1);
	pros::Motor back_right_mtr(20);
	pros::Motor back_left_mtr(11);
	// pros::Motor arm_mtr(12);

	// while (true)
	// {
	// 	//Get Joystick Values
	// 	int joystickCh1 = controller.get_analog(ANALOG_RIGHT_X);
	// 	int joystickCh3 = controller.get_analog(ANALOG_LEFT_Y);
	// 	int joystickCh4 = controller.get_analog(ANALOG_LEFT_X);

	// 	// Motor speed control
	// 	front_right_mtr.move(-joystickCh3 + joystickCh1 + joystickCh4);
	// 	front_left_mtr.move(joystickCh3 + joystickCh1 + joystickCh4);
	// 	back_right_mtr.move(-joystickCh3 + joystickCh1 - joystickCh4);
	// 	back_left_mtr.move(joystickCh3 + joystickCh1 - joystickCh4);

	// 	// pros::lcd::set_text(7, std::to_string(theta));
	// 	pros::delay(20);
	// }

	rotateToAngle(1080);
}