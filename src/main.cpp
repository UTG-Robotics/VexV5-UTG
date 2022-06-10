#include "main.h"
#include "autoSelect/selection.h"
#include <fstream>

/* Build with
pros make -- FLAGS=-DREPLAY
to replay, will record otherwise

*/

// Change this comment to reupload

// #define REPLAY

#ifndef REPLAY
#define RECORD
#endif

double xPos = 0;
double yPos = 0;
double angle = 0;
int selectedAuto = 0;
bool hasAutoStarted = false;
bool startReplay = false;

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::IMU gyro(3);
pros::Rotation leftEncoder(11);
pros::Rotation rightEncoder(20);
pros::Rotation sideEncoder(7);

pros::Motor front_right_mtr(6);
pros::Motor front_left_mtr(5);
pros::Motor back_right_mtr(16);
pros::Motor back_left_mtr(15);
pros::Motor arm_mtr_left(1);
pros::Motor arm_mtr_right(10);
pros::Motor claw_mtr(8);
pros::Motor back_claw_mtr(9);

pros::Motor spinner(4);

Arm arm(1, 10, 1);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void autoSelector()
{
	std::string autos[5] = {"Left Win Point", "Right Win Point", "Left Center Goal", "Right Center Goal", "Skills Autonomous"};
	selectedAuto = 0;
	std::string autoString = autos[selectedAuto];
	controller.clear();
	pros::delay(50);
	controller.set_text(0, ceil((19 - autoString.length()) / 2), autoString);
	while (!hasAutoStarted)
	{
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && selectedAuto > 0)
		{
			selectedAuto--;
			controller.clear();
			autoString = autos[selectedAuto];
			pros::delay(50);
			controller.set_text(0, ceil((20 - autoString.length()) / 2), autoString);
			pros::delay(50);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) && selectedAuto < 4)
		{
			selectedAuto++;
			controller.clear();
			autoString = autos[selectedAuto];
			pros::delay(50);
			controller.set_text(0, ceil((20 - autoString.length()) / 2), autoString);
			pros::delay(50);
		}
		pros::delay(20);
		// printf("%d\n", selectedAuto);
	}
}

void initialize()
{
	pros::Task odometry_task(odometry);
	pros::Task selector_task(autoSelector);

	selector::init();
	arm_mtr_left.set_reversed(true);
	claw_mtr.set_reversed(true);
	back_claw_mtr.set_reversed(true);
	pros::delay(2000);
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
void competition_initialize()
{
}

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

/*
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
*/
void autonomous()
{
	leftEncoder.reset();
	rightEncoder.reset();
	sideEncoder.reset();
	leftEncoder.reset_position();
	rightEncoder.reset_position();
	sideEncoder.reset_position();

	// selectedAuto = 0;
	hasAutoStarted = true;
}

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
	// pros::delay(5000);
	// autonomous();
	bool fieldCentric = false;
	double moveSpeed = 0;
	double rotSpeed = 0;

	leftEncoder.reset();
	rightEncoder.reset();
	sideEncoder.reset();
	leftEncoder.reset_position();
	rightEncoder.reset_position();
	sideEncoder.reset_position();

	xPos = 0;
	yPos = 0;
	angle = 0;

	arm.setMode(false);
	arm_mtr_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	arm_mtr_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	claw_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	back_claw_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	spinner.move(127);

	while (true)
	{
		float joystickCh1 = controller.get_analog(ANALOG_RIGHT_X) / 127.0 * 200.0;
		float joystickCh2 = controller.get_analog(ANALOG_RIGHT_Y) / 127.0 * 200.0;
		float joystickCh3 = controller.get_analog(ANALOG_LEFT_Y) / 127.0 * 200.0;
		float joystickCh4 = controller.get_analog(ANALOG_LEFT_X) / 127.0 * 200.0;

		if (!fieldCentric)
		{
			front_right_mtr.move_velocity(-joystickCh3 + joystickCh1 + joystickCh4);
			front_left_mtr.move_velocity(joystickCh3 + joystickCh1 + joystickCh4);
			back_right_mtr.move_velocity(-joystickCh3 + joystickCh1 - joystickCh4);
			back_left_mtr.move_velocity(joystickCh3 + joystickCh1 - joystickCh4);
		}
		else
		{
			// driveToPoint(0, 0, 3600, 127);
			double targetAngle = (atan2(-joystickCh4, joystickCh3) + M_PI / 2 + angle);

			double xRatio = -cos(targetAngle + (M_PI / 4));
			double yRatio = sin(targetAngle + (M_PI / 4));

			// Normalize values to maximum of 1
			double maxRatio = std::max(abs(xRatio), abs(yRatio));
			double xPowerPercentage = (xRatio / maxRatio);
			double yPowerPercentage = (yRatio / maxRatio);

			double joystickMag = hypot(joystickCh4, joystickCh3);

			moveSpeed = joystickMag;
			rotSpeed = joystickCh1;

			// Move at angle while rotating
			front_right_mtr.move(-xPowerPercentage * moveSpeed + rotSpeed);
			front_left_mtr.move(yPowerPercentage * moveSpeed + rotSpeed);
			back_right_mtr.move(-yPowerPercentage * moveSpeed + rotSpeed);
			back_left_mtr.move(xPowerPercentage * moveSpeed + rotSpeed);
		}
		pros::delay(20);
	}
}