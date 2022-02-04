#include "main.h"

double xPos = 0;
double yPos = 0;
double angle = 0;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize()
{
	pros::Task odometry_task(odometry);
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
Our auton, who art in c++,
    called by thy name;
  thy operating system run;
thy program run correctly
  as it did in testing.
Give us this day our daily miracle.
And forgive us our sketchy autonomous,
    as we forgive Cooper
  who writes broken code.
And lead us not into defeat;
  but deliver us from coding at the tournament.
    For thine is the awp,
the odometry, and the tracking,
  leading us, our robot, 
    to victory.
      Return;
**/

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

	pros::Rotation leftEncoder(11);
	pros::Rotation rightEncoder(20);
	pros::Rotation sideEncoder(7);
	pros::IMU gyro(13);
	// gyro.reset();

	printf("gyro reset");
	for (int i = 0; i < 1; i++)
	{
		leftEncoder.reset();
		rightEncoder.reset();
		sideEncoder.reset();
		leftEncoder.reset_position();
		rightEncoder.reset_position();
		sideEncoder.reset_position();
		xPos = 0;
		yPos = 0;
		angle = 0;
		pros::delay(100);
	}

	// Initialize the drivetrains
	pros::Controller controller(pros::E_CONTROLLER_MASTER);
	pros::Motor front_right_mtr(6);
	pros::Motor front_left_mtr(5);
	pros::Motor back_right_mtr(16);
	pros::Motor back_left_mtr(15);
	// pros::Motor arm_mtr(9);

	// front_right_mtr.move_velocity(-50);
	// back_left_mtr.move_velocity(50);

	// pros::delay(7500);

	// front_right_mtr.move_velocity(0);
	// back_left_mtr.move_velocity(0);

	// front_left_mtr.move_velocity(50);
	// front_right_mtr.move_velocity(50);
	// back_left_mtr.move_velocity(-50);
	// back_right_mtr.move_velocity(-50);

	// pros::delay(5000);

	// front_left_mtr.move_velocity(0);
	// front_right_mtr.move_velocity(0);
	// back_left_mtr.move_velocity(0);
	// back_right_mtr.move_velocity(0);

	// arm_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	pros::lcd::set_text(1, "Calibrated");
	// pros::delay(3000);
	driveToPoint(0, 0, 180);
	// rotateToAngle(90);
	// driveToPoint(0, 0, 0);
	while (true)
	{
		//Get Joystick Values
		int joystickCh1 = controller.get_analog(ANALOG_RIGHT_X);
		int joystickCh3 = controller.get_analog(ANALOG_LEFT_Y);
		int joystickCh4 = controller.get_analog(ANALOG_LEFT_X);

		//Control arm
		// if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		// {
		// 	arm_mtr.move(50);
		// }
		// else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		// {
		// 	arm_mtr.move(-127);
		// }
		// else
		// {
		// 	arm_mtr.move(0);
		// }

		// Motor speed control
		front_right_mtr.move(-joystickCh3 + joystickCh1 + joystickCh4);
		front_left_mtr.move(joystickCh3 + joystickCh1 + joystickCh4);
		back_right_mtr.move(-joystickCh3 + joystickCh1 - joystickCh4);
		back_left_mtr.move(joystickCh3 + joystickCh1 - joystickCh4);

		pros::delay(20);
	}
}
