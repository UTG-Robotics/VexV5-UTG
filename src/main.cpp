#include "main.h"
#include "autoSelect/selection.h"
#include <fstream>
#include <iomanip>

double xPos = 0;
double yPos = 0;
double angle = 0;
int selectedAuto = 0;
bool hasAutoStarted = false;
bool isShooting = false;
int goal = 2300;
bool isSpinning = true;
bool isTurretMode = false;
bool isRollerMode = false;
bool isForward = false;
int counter = 0;

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Motor front_right_mtr(11);
pros::Motor front_left_mtr(20);
pros::Motor back_right_mtr(10);
pros::Motor back_left_mtr(2);

// pros::Motor flywheel_mtr(8);
// pros::Motor flywheel_mtr_two(7);
pros::Motor intake_mtr(1);
pros::Motor indexer_mtr(19);
pros::Motor expansion_mtr(17);

auto flywheel_motor = sylib::Motor(8, 3000, false);
auto flywheel_motor_two = sylib::Motor(7, 3000, true);
// 3.5417x+1086
VelPID *drivePID = new VelPID(5, 0, 0, 3.5417, 1086, 0.1, false);
Flywheel flywheel(&flywheel_motor, &flywheel_motor_two, drivePID, new EMAFilter(0.15), 15, 50);
XDrive driveTrain(&front_right_mtr, &front_left_mtr, &back_right_mtr, &back_left_mtr, 20);
Indexer indexer(&indexer_mtr);
Piston expansion(1);
PID autoAimPID = PID(0.015, 0.001, 0.005, 5);

// std::shared_ptr<OdomChassisController> chassis =
// 	ChassisControllerBuilder()
// 		.withMotors(6, 6) // left motor is 1, right motor is 2 (reversed)
// 		// green gearset, 4 inch wheel diameter, 11.5 inch wheel track
// 		.withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
// 		// left encoder in ADI ports A & B, right encoder in ADI ports C & D (reversed)
// 		.withSensors(okapi::RotationSensor{3}, okapi::RotationSensor{12}, okapi::RotationSensor{18, true})
// 		// specify the tracking wheels diameter (2.75 in), track (7 in), and TPR (360)
// 		.withOdometry({{2.75_in, 13.8976378_in, 6.4370079_in, 2.75_in}, 360})
// 		.buildOdometry();

// std::shared_ptr<okapi::OdomChassisController> chassis =
// 	okapi::ChassisControllerBuilder()
// .withSensors(okapi::RotationSensor{3}, okapi::RotationSensor{12}, okapi::RotationSensor{18, true})
// // specify the tracking wheels diameter (2.75 in), track (7 in), and TPR (360)
// .buildOdometry();

// Flywheel flywheel(&flywheel_mtr, new VelPID(0, 0, 0, 3.95, 242, 0.1), new EMAFilter(0.15), 15, 50);

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
	sylib::initialize();
	selector::init();
	pros::lcd::initialize();
	pros::delay(2000);
	// gyro.reset();
	pros::Task odometry_task(odometry);
	leftEncoder.reset();
	rightEncoder.reset();
	sideEncoder.reset();
	leftEncoder.reset_position();
	rightEncoder.reset_position();
	sideEncoder.reset_position();
	xPos = 0;
	yPos = 0;
	angle = 0;
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
	/*
	hasAutoStarted = true;
	printf("Auto Started\n");

	// selector::auton = 2;
	// flywheel.updatePID(drivePID);
	// Right Auto
	if (selector::auton == 3)
	{
		driveTrain.setStartPos(84, 14, 0);
		// Drive sideways to roller
		driveTrain.driveToPoint(108, 14, 0, 100, 2500);
		flywheel.setTargetRPM(2780);
		// Drive into roller
		driveTrain.driveToPoint(108, 9, 0, 100, 1000);
		// Spin roller
		intake_mtr.move_velocity(-50);
		pros::delay(500);
		intake_mtr.move_velocity(0);
		// Drive back + turn to goal
		driveTrain.driveToPoint(108, 14, 8.5, 100, 1000);
		// Take two shots
		flywheel.waitUntilReady();
		indexer.shoot();
		pros::delay(250);
		flywheel.waitUntilReady();
		indexer.shoot();
		pros::delay(1000);
	}
	// Left Auto
	else if (selector::auton == 2)
	{
		driveTrain.setStartPos(108, 14, 0);
		flywheel.setTargetRPM(2780);
		// Drive into roller
		driveTrain.driveToPoint(106, 8, 0, 100, 1000);
		// Spin roller
		intake_mtr.move_velocity(-50);
		pros::delay(500);
		intake_mtr.move_velocity(0);
		// Drive back + turn to goal
		driveTrain.driveToPoint(106, 14, -6, 100, 1000);
		// Take two shots
		flywheel.waitUntilReady();
		indexer.shoot();
		pros::delay(250);
		flywheel.waitUntilReady();
		indexer.shoot();
		pros::delay(1000);
		// driveTrain.driveToPoint(96, 24, 0, 100, 3000);
	}

	printf("Auto Ended\n");
	*/
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

// OdomState getOdomState()
// {
// 	return {chassis->getState().x.convert(okapi::inch), chassis->getState().y.convert(okapi::inch), chassis->getState().theta.convert(okapi::degree)};
// }
void opcontrol()
{
	// leftEncoder.reset();
	// rightEncoder.reset();
	// sideEncoder.reset();
	// leftEncoder.reset_position();
	// rightEncoder.reset_position();
	// sideEncoder.reset_position();
	// autonomous();
	// flywheel.updatePID(drivePID);

	// while (goal <= 13000)
	// {
	// 	flywheel.setTargetRPM(goal);
	// 	pros::delay(5000);
	// 	printf("\nKv: %d|%f,%f", goal, flywheel_motor.get_velocity(), flywheel_motor_two.get_velocity());
	// 	goal += 1000;
	// }

	// pros::delay(5000);
	double moveSpeed = 0;
	double rotSpeed = 0;

	// leftEncoder.reset();
	// rightEncoder.reset();
	// sideEncoder.reset();
	// leftEncoder.reset_position();
	// rightEncoder.reset_position();
	// sideEncoder.reset_position();

	// arm_mtr_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	// arm_mtr_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	// claw_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	// back_claw_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	// Shoot is R1
	// L1 intake forward. L2 backwards

	// driveTrain.driveToPoint(-12, -12, 90, 127, 2500000);
	// driveTrain.setStartPos(80.25, 14.75, 0);
	driveTrain.setStartPos(0, 0, 0);
	// xPos = 80.25;
	// yPos = 14.75;
	// angle = 0;
	xPos = 0;
	yPos = 0;
	angle = 0;

	intake_mtr.move_velocity(130);
	while (true)
	{
		float joystickCh1 = controller.get_analog(ANALOG_RIGHT_X) / 127.0 * 200.0;
		float joystickCh2 = controller.get_analog(ANALOG_RIGHT_Y) / 127.0 * 200.0;
		float joystickCh3 = controller.get_analog(ANALOG_LEFT_Y) / 127.0 * 200.0;
		float joystickCh4 = controller.get_analog(ANALOG_LEFT_X) / 127.0 * 200.0;

		if (controller.get_digital(DIGITAL_R1) && flywheel.IsAtTarget())
		{
			indexer.shoot();
		}

		if (controller.get_digital_new_press(DIGITAL_R2))
		{
			isRollerMode = !isRollerMode;
		}

		if (controller.get_digital(DIGITAL_L1))
		{
			// spin intake forwards
			intake_mtr.move_velocity(200 * (isRollerMode ? 0.5 : 1));
		}
		else if (controller.get_digital(DIGITAL_L2))
		{
			// spin intake backwards
			intake_mtr.move_velocity(-200 * (isRollerMode ? 0.5 : 1));
		}

		if (controller.get_digital_new_press(DIGITAL_X))
		{
			isTurretMode = !isTurretMode;
		}

		if (controller.get_digital_new_press(DIGITAL_A))
		{
			isSpinning = !isSpinning;
		}

		if (controller.get_digital_new_press(DIGITAL_B))
		{
			isForward = !isForward;
		}

		if (controller.get_digital(DIGITAL_Y))
		{
			expansion_mtr.move_velocity(40);
		}
		else
		{
			expansion_mtr.move_velocity(0);
		}

		if (controller.get_digital_new_press(DIGITAL_B))
		{
			isForward = !isForward;
		}

		if (controller.get_digital_new_press(DIGITAL_UP))
		{
			goal += 100;
		}
		else if (controller.get_digital_new_press(DIGITAL_DOWN))
		{
			goal -= 100;
		}

		if (controller.get_digital_new_press(DIGITAL_RIGHT))
		{
			expansion.toggle();
		}

		flywheel.setTargetRPM(isSpinning ? goal : 0);
		// flywheel_motor_two.set_velocity_custom_controller(isSpinning ? goal : 0);
		// flywheel_motor.set_velocity_custom_controller(isSpinning ? goal : 0);

		if (isTurretMode)
		{
			joystickCh1 /= 5;
		}

		double target = atan2(xPos - 19, yPos - 19) * 180 / M_PI - 180;
		// printf("\n%f,%f,%f,%f", target, xPos, yPos, angle);
		driveTrain.arcade(joystickCh4 * (isForward ? -1 : 1), joystickCh3 * (isForward ? -1 : 1), joystickCh1);
		// driveTrain.arcade(joystickCh4 * (isForward ? -1 : 1), joystickCh3 * (isForward ? -1 : 1), autoAimPID.calculate(target - angle * 180 / M_PI) * 127);

		// driveTrain.arcade(joystickCh4 * (isForward ? -1 : 1), joystickCh3 * (isForward ? -1 : 1), autoAimPID.calculate(target - angle * 180 / M_PI) * 127);

		if (counter % 60 == 0)
		{
			controller.clear_line(1);
		}
		if (counter % 30 == 0)
		{
			controller.clear_line(0);
		}
		if (counter % 15 == 0)
		{
			controller.set_text(1, 0, "Goal: " + std::to_string((int)goal));
		}
		if (counter % 5 == 0)
		{
			controller.set_text(0, 0, "RPM: " + std::to_string((int)flywheel_motor.get_velocity()));
		}

		// printf("X: %f, Y: %f, Angle: %f", getOdomState().x, getOdomState().y, getOdomState().theta);

		counter++;
		pros::delay(20);
	}
}