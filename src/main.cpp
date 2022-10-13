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
int counter = 0;

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Motor front_right_mtr(11);
pros::Motor front_left_mtr(20);
pros::Motor back_right_mtr(10);
pros::Motor back_left_mtr(2);

pros::Motor flywheel_mtr(8);
pros::Motor intake_mtr(1);
pros::Motor indexer_mtr(19);

Flywheel flywheel(&flywheel_mtr, new VelPID(1, 0.1, 0, 3.95, 242, 0.1), new EMAFilter(0.15), 15, 50);
XDrive driveTrain(&front_right_mtr, &front_left_mtr, &back_right_mtr, &back_left_mtr, 20);
std::shared_ptr<OdomChassisController> chassis =
	ChassisControllerBuilder()
		.withMotors(6, 6) // left motor is 1, right motor is 2 (reversed)
		// green gearset, 4 inch wheel diameter, 11.5 inch wheel track
		// left encoder in ADI ports A & B, right encoder in ADI ports C & D (reversed)
		.withSensors(okapi::RotationSensor{3}, okapi::RotationSensor{12}, okapi::RotationSensor{18, true})
		// specify the tracking wheels diameter (2.75 in), track (7 in), and TPR (360)
		.withOdometry({{2.75_in, 13.8976378_in, 6.4370079_in, 2.75_in}, 360})
		.buildOdometry();

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
	// selector::init();
	pros::lcd::initialize();
	pros::delay(2000);
	chassis->setState({0_in, 0_in, 0_deg});
	// pros::Task odometry_task(odometry);
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

// OdomState getOdomState()
// {
// 	return {chassis->getState().x.convert(okapi::inch), chassis->getState().y.convert(okapi::inch), chassis->getState().theta.convert(okapi::degree)};
// }
void opcontrol()
{
	// pros::delay(5000);
	// autonomous();
	double moveSpeed = 0;
	double rotSpeed = 0;

	// leftEncoder.reset();
	// rightEncoder.reset();
	// sideEncoder.reset();
	// leftEncoder.reset_position();
	// rightEncoder.reset_position();
	// sideEncoder.reset_position();

	xPos = 0;
	yPos = 0;
	angle = 0;

	indexer_mtr.set_reversed(true);
	// arm_mtr_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	// arm_mtr_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	// claw_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	// back_claw_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	// Shoot is R1
	// L1 intake forward. L2 backwards

	intake_mtr.move_velocity(130);
	chassis->setState({0_in, 0_in, 0_deg});
	while (true)
	{
		float joystickCh1 = controller.get_analog(ANALOG_RIGHT_X) / 127.0 * 200.0;
		float joystickCh2 = controller.get_analog(ANALOG_RIGHT_Y) / 127.0 * 200.0;
		float joystickCh3 = controller.get_analog(ANALOG_LEFT_Y) / 127.0 * 200.0;
		float joystickCh4 = controller.get_analog(ANALOG_LEFT_X) / 127.0 * 200.0;

		if (controller.get_digital_new_press(DIGITAL_R1))
		{
			// shoot
			isShooting = true;
			indexer_mtr.tare_position();
			indexer_mtr.move_absolute(100, 200);
		}
		if (isShooting)
		{
			if (indexer_mtr.get_position() - 100 > -20)
			{
				isShooting = false;
				indexer_mtr.move_absolute(0, 200);
			}
		}

		if (controller.get_digital(DIGITAL_L1))
		{
			// spin intake forwards
			intake_mtr.move_velocity(130);
		}
		else if (controller.get_digital(DIGITAL_L2))
		{
			// spin intake backwards
			intake_mtr.move_velocity(-130);
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

		flywheel.setTargetRPM(isSpinning ? goal : 0);
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
		pros::lcd::clear_line(3);

		std::stringstream ss;
		ss << std::fixed << std::setprecision(2) << "X: " << chassis->getState().x.convert(inch) << " Y: " << chassis->getState().y.convert(inch) << " Theta: " << chassis->getState().theta.convert(degree);

		// pros::lcd::set_text(3, "X: " + std::to_string(chassis->getState().x.convert(inch)) + " Y: " + std::to_string(chassis->getState().y.convert(inch)) + " Theta: " + std::to_string(chassis->getState().theta.convert(degree)));
		pros::lcd::set_text(3, ss.str());

		driveTrain.arcade(joystickCh4, joystickCh3, joystickCh1);

		// printf("X: %f, Y: %f, Angle: %f", getOdomState().x, getOdomState().y, getOdomState().theta);
		counter++;
		pros::delay(20);
	}
}