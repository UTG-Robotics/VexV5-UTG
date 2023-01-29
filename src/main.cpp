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
int goal = 3000;
bool isSpinning = false;
bool isRollerMode = false;
int intakeDir = 1;
int counter = 0;

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor front_right_mtr(13);
pros::Motor front_left_mtr(20);
pros::Motor back_right_mtr(1);
pros::Motor back_left_mtr(8);

pros::Motor_Group left_motors({front_left_mtr, back_left_mtr});
pros::Motor_Group right_motors({front_right_mtr, back_right_mtr});

pros::Motor intake_mtr(10);
pros::Motor indexer_mtr(9);

auto flywheel_motor = sylib::Motor(11, 3600, false);
auto flywheel_motor_two = sylib::Motor(12, 3600, true);
// 3.0062x+584.13
VelPID *drivePID = new VelPID(7, 0, 0, 3.0062, 584.13, 0.1, false);
Flywheel flywheel(&flywheel_motor, &flywheel_motor_two, drivePID, new EMAFilter(0.15), 15, 50);
TankDrive driveTrain(&left_motors, &right_motors);

Indexer indexer(&indexer_mtr);
pros::ADIPotentiometer potentiometer('F', pros::E_ADI_POT_V2);
Piston expansion(1);

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
	driveTrain.setBrake(0.02, 10);
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
 */
void competition_initialize()
{
}

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

void opcontrol()
{
	// goal = 1000;
	// while (goal <= 13000)
	// {
	// 	flywheel.setTargetRPM(goal);
	// 	pros::delay(5000);
	// 	printf("\nKv: %d|%f,%f", goal, flywheel_motor.get_velocity(), flywheel_motor_two.get_velocity());
	// 	goal += 1000;
	// }

	double moveSpeed = 0;
	double rotSpeed = 0;

	xPos = 0;
	yPos = 0;
	angle = 0;

	// std::vector<double> profile = generateProfile(0, 0.304, 1, 5, 10, 0.001);
	// driveTrain.followProfileForward(profile);

	while (true)
	{
		float joystickCh1 = controller.get_analog(ANALOG_RIGHT_X) / 127.0 * 200.0;
		float joystickCh2 = controller.get_analog(ANALOG_RIGHT_Y) / 127.0 * 200.0;
		float joystickCh3 = controller.get_analog(ANALOG_LEFT_Y) / 127.0 * 200.0;
		float joystickCh4 = controller.get_analog(ANALOG_LEFT_X) / 127.0 * 200.0;

		if (controller.get_digital(DIGITAL_R1))
		{
			indexer.shoot();
		}

		if (controller.get_digital_new_press(DIGITAL_X))
		{
			isRollerMode = !isRollerMode;
		}

		if (controller.get_digital(DIGITAL_L1))
		{
			// spin intake forwards
			intakeDir = 1;
		}
		else if (controller.get_digital(DIGITAL_L2))
		{
			// spin intake backwards
			intakeDir = -1;
		}

		if (controller.get_digital_new_press(DIGITAL_A))
		{
			isSpinning = !isSpinning;
		}

		if (controller.get_digital_new_press(DIGITAL_UP))
		{
			goal += 100;
		}
		else if (controller.get_digital_new_press(DIGITAL_DOWN))
		{
			goal -= 100;
		}

		if (controller.get_digital(DIGITAL_RIGHT) && controller.get_digital(DIGITAL_Y))
		{
			expansion.toggle();
		}

		flywheel.setTargetRPM(isSpinning ? goal : 0);

		intake_mtr.move_velocity(intakeDir * (isRollerMode ? 0.5 : 1) * 200);
		driveTrain.tank(joystickCh3, joystickCh2);

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
			controller.set_text(0, 0, "RPM: " + std::to_string((int)flywheel.getCurrentRPM()));
		}

		counter++;
		pros::delay(20);
	}
}