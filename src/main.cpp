#include "main.h"
#include "autoSelect/selection.h"
#include <fstream>
// #include <iomanip>

double xPos = 0;
double yPos = 0;
double angle = 0;
int selectedAuto = 0;
bool hasAutoStarted = false;
bool isShooting = false;
int goal = 2300;
bool isSpinning = true;
bool isRollerMode = false;
int intakeDir = 1;
int counter = 0;

int disabledCounter = 0;
bool driverStarted = false;

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor front_right_mtr(13);
pros::Motor front_left_mtr(20);
pros::Motor back_right_mtr(1);
pros::Motor back_left_mtr(8);

pros::Motor_Group left_motors({front_left_mtr, back_left_mtr});
pros::Motor_Group right_motors({front_right_mtr, back_right_mtr});

pros::Motor intake_mtr(10);
pros::Motor indexer_mtr(9);

pros::Imu imu(4);
auto flywheel_motor = sylib::Motor(11, 3600, false);
auto flywheel_motor_two = sylib::Motor(12, 3600, true);
// 3.0062x+584.13
VelPID *drivePID = new VelPID(7, 0, 0.01, 3.0062, 584.13, 0.1, false);
Flywheel flywheel(&flywheel_motor, &flywheel_motor_two, drivePID, new EMAFilter(0.15), 15, 50);
TankDrive driveTrain(&left_motors, &right_motors, &imu);

pros::ADIPotentiometer potentiometer('F', pros::E_ADI_POT_V2);
Indexer indexer(&indexer_mtr, &potentiometer);

Piston expansion(1);

pros::ADIAnalogOut coarse(1);
pros::ADIAnalogOut fine(2);

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
		printf("%d\n", selectedAuto);
	}
}

std::pair<int, int> HzToCommand(int hz)
{
	std::vector<std::pair<int, int>> freqLookup = {{52, 18}, {101, 21}, {152, 25}, {202, 28}, {254, 32}, {306, 35}, {358, 40}, {410, 43}, {462, 46}, {515, 50}, {566, 53}, {614, 57}, {665, 62}, {717, 65}, {767, 68}, {815, 72}, {866, 76}, {919, 80}, {970, 83}, {1020, 87}, {1071, 90}, {1124, 94}, {1175, 98}, {1227, 102}, {1278, 105}, {1331, 109}, {1382, 112}, {1432, 116}, {1483, 120}, {1535, 124}, {1585, 128}, {1634, 131}, {1685, 135}, {1737, 138}, {1788, 142}, {1839, 146}, {1891, 149}, {1944, 153}, {1995, 156}, {2047, 160}, {2099, 164}, {1252, 167}, {2203, 171}, {2252, 175}, {2303, 179}, {2355, 182}, {2405, 186}, {2454, 190}, {2507, 193}, {2558, 197}, {2610, 201}, {2661, 204}, {2712, 208}, {2765, 212}, {2818, 215}, {2869, 219}, {2920, 223}, {2974, 226}, {3026, 230}, {3077, 234}, {3130, 237}, {3206, 240}, {3448, 244}, {4600, 248}, {8120, 251}, {0, 255}};
	// Loop through the lookup table and find the lowest frequency that is greater than the input
	if (hz <= 81)
	{
		return {255, 255};
	}

	int coarse = 0;
	int diff = 0;
	int fine = 0;

	for (int i = 0; i < freqLookup.size(); i++)
	{
		if (freqLookup.at(i).first > hz)
		{
			coarse = freqLookup.at(i - 1).second;
			diff = hz - freqLookup.at(i - 1).first;
			break;
		}
	}
	fine = freqLookup.at(diff).second;

	return {coarse, fine};
}
void playFreq(int hz)
{
	std::pair<int, int> command = HzToCommand(hz);
	coarse.set_value(command.first);
	fine.set_value(command.second);
}

void initialize()
{
	std::cout << "Initializing" << std::endl;
	sylib::initialize();
	selector::init();
	playFreq(0);
	// pros::lcd::initialize();
	pros::delay(2000);
	driveTrain.setBrake(0.01);
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
void disabled()
{
	disabledCounter++;
	if (disabledCounter >= 1 && driverStarted)
	{

		Sound sound("Jeopardy:d=4,o=6,b=125:c,f,c,f5,c,f,2c,c,f,c,f,a.,8g,8f,8e,8d,8c#,c,f,c,f5,c,f,2c,f.,8d,c,a#5,a5,g5,f5,p,d#,g#,d#,g#5,d#,g#,2d#,d#,g#,d#,g#,c.7,8a#,8g#,8g,8f,8e,d#,g#,d#,g#5,d#,g#,2d#,g#.,8f,d#,c#,c,p,a#5,p,g#.5,d#,g#");
		for (int i = 0; i < sound.notes.size(); i++)
		{
			Note note = sound.notes.at(i);
			playFreq(note.hz);
			pros::delay(note.duration);
			playFreq(0);
			pros::delay(20);
		}
	}
}

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
	playFreq(0);
	/*
		// Right side
		driveTrain.forwardPID(-18, 127 / 2, true);
		driveTrain.autoWait();
		driveTrain.turnPID(90, 127 / 2);
		driveTrain.autoWait();
		flywheel.setTargetRPM(2960);
		driveTrain.forwardPID(-3, 127 / 2, true);
		driveTrain.autoWait();
		intake_mtr.move_velocity(-150);
		driveTrain.tank(-30, -30);
		pros::delay(750);
		intake_mtr.move_velocity(0);
		driveTrain.forwardPID(6, 127 / 2, true);
		driveTrain.autoWait();
		driveTrain.turnPID(97, 127 / 2);
		driveTrain.autoWait();
		indexer.shoot(true);
		indexer.shoot(true);
		driveTrain.turnPID(225, 127 / 2);
		driveTrain.autoWait();
		intake_mtr.move_velocity(200);
		driveTrain.forwardPID(-60, 55, true);
		driveTrain.autoWait();
		flywheel.setTargetRPM(2700);
		driveTrain.turnPID(136, 127 / 2);
		driveTrain.autoWait();
		indexer.shoot(true);
		indexer.shoot(true);
		indexer.shoot(true);
		*/

	/*
	// Left side
	driveTrain.tank(-30, -30);
	intake_mtr.move_velocity(-150);
	pros::delay(250);
	intake_mtr.move_velocity(0);
	flywheel.setTargetRPM(3030);
	driveTrain.swingPID(-10, 127 / 2, false);
	driveTrain.autoWait();
	pros::delay(1000);
	indexer.shoot(true);
	indexer.shoot(true);
	pros::delay(1000);
	driveTrain.turnPID(-135, 127 / 2);
	driveTrain.autoWait();
	pros::delay(1000);
	intake_mtr.move_velocity(200);
	flywheel.setTargetRPM(2800);
	driveTrain.forwardPID(-38, 127 * 0.75, true);
	driveTrain.autoWaitUntil(25);
	driveTrain.setMaxSpeed(127 / 4);
	driveTrain.autoWaitUntil(10);
	driveTrain.setMaxSpeed(127 / 6);
	driveTrain.autoWait();
	driveTrain.turnPID(-28, 127 / 2);
	driveTrain.autoWait();
	pros::delay(1000);
	indexer.shoot(true);
	indexer.shoot(true);
	indexer.shoot(true);
	*/
	hasAutoStarted = true;
	selector::auton = 3;

	// flywheel.updatePID(drivePID);
	// Right Auto
	if (selector::auton == 3)
	{

		// driveTrain.followProfileForward(generateProfile(0, -60 / 12.0, 4, 20.25, 40, 0.0001));

		// while (1)
		// {
		// 	pros::delay(20);
		// }

		driveTrain.autoOver = true;
		flywheel.setTargetRPM(3500);

		driveTrain.forwardPID(-22, 127 / 2, true);
		driveTrain.autoWait();
		driveTrain.turnPID(90, 127 / 2);
		driveTrain.autoWait();
		// driveTrain.followProfileForward(generateProfile(0, - / 12.0, 4, 20.25, 40, 0.0001));
		driveTrain.forwardPID(-3, 127 / 2, true);
		driveTrain.autoWait();
		driveTrain.tank(-20, -20);
		pros::delay(500);
		// intake_mtr.move_relative(-500, 200);
		intake_mtr.move_absolute(-500, 100);
		// while (abs(intake_mtr.get_position() - -500) > 20)
		// {
		// 	pros::delay(20);
		// }
		pros::delay(350);
		driveTrain.forwardPID(3, 127 / 2, true);
		driveTrain.autoWait();
		// driveTrain.followProfileForward(generateProfile(0, 1, 4, 20.25, 40, 0.0001));
		// driveTrain.followProfileForward(generateProfile(0, 0.05, 4, 20.25, 40, 0.0001));
		// pros::delay(100);
		driveTrain.turnPID(92, 127 / 2);
		driveTrain.autoWait();
		pros::delay(100);
		indexer.shoot(true);
		pros::delay(700);
		indexer.shoot(true);
		pros::delay(100);
		driveTrain.turnPID(225, 127 / 2);
		driveTrain.autoWait();
		intake_mtr.move_velocity(200);
		// driveTrain.followProfileForward(generateProfile(0, -60 / 12.0, 4, 20.25, 40, 0.0001));
		flywheel.setTargetRPM(3150);
		driveTrain.forwardPID(-70, 127 / 2.5, true);
		driveTrain.autoWait();
		driveTrain.turnPID(129, 127 / 2);
		driveTrain.autoWait();
		indexer.shoot(true);
		pros::delay(350);
		indexer.shoot(true);
		pros::delay(350);
		indexer.shoot(true);
	}
	// Left Auto
	else if (selector::auton == 2)
	{
		driveTrain.autoOver = true;
		flywheel.setTargetRPM(3400);
		pros::delay(2000);
		driveTrain.followProfileForward(generateProfile(0, -0.2, 4, 20.25, 40, 0.0001));
		// driveTrain.followProfileForward(generateProfile(0, -0.2, 4, 20.25, 40, 0.0001));
		driveTrain.tank(-20, -20);
		intake_mtr.move_absolute(-1000, 100);
		pros::delay(250);
		driveTrain.tank(0, 0);
		driveTrain.swingPID(-16, 127 / 2, false);
		driveTrain.autoWait();
		// pros::delay(250);
		indexer.shoot(true);
		pros::delay(350);
		indexer.shoot(true);
		pros::delay(250);

		driveTrain.turnPID(-128, 127 / 2);
		driveTrain.autoWait();
		// pros::delay(1000);
		intake_mtr.move_velocity(200);
		flywheel.setTargetRPM(3100);
		// driveTrain.forwardPID(-38, 127 * 0.75, false);
		// driveTrain.followProfileForward(generateProfile(0, -13.0 / 12.0, 3.5, 20.25, 40, 0.0001));
		driveTrain.forwardPID(-35, 127 * 0.75, true);
		driveTrain.autoWait(5);
		// driveTrain.followProfileForward(generateProfile(0, -4 / 12.0, 2, 20.25, 40, 0.0001));
		// pros::delay(250);
		driveTrain.forwardPID(-30, 127 * 0.25, true);
		driveTrain.autoWait();
		// driveTrain.followProfileForward(generateProfile(0, -6.0 / 12.0, 0.5, 20.25, 40, 0.0001));
		// driveTrain.autoWaitUntil(26);
		// driveTrain.setMaxSpeed(127 / 4);
		// driveTrain.autoWaitUntil(12);
		// driveTrain.setMaxSpeed(127 / 6);
		// driveTrain.autoWait();
		driveTrain.turnPID(-39, 127 / 2);
		driveTrain.autoWait();
		pros::delay(300);
		indexer.shoot(true);
		pros::delay(250);
		indexer.shoot(true);
		pros::delay(250);
		indexer.shoot(true);
	}
	else if (selector::auton == 1)
	{
		driveTrain.autoOver = true;
	}
	else if (selector::auton == 0)
	{
		driveTrain.startAuto();
		driveTrain.tank(-30, -30);
		pros::delay(100);
		intake_mtr.move_velocity(-200);
		pros::delay(350);
		intake_mtr.move_velocity(0);
		driveTrain.forwardPID(3, 127 / 2, true);
		driveTrain.autoWaitUntil(1);
		driveTrain.turnPID(135, 127 / 3);
		driveTrain.autoWait();
		intake_mtr.move_velocity(200);
		driveTrain.forwardPID(-18, 127 / 2.5, true);
		driveTrain.autoWait();
		driveTrain.turnPID(90, 127 / 2);
		flywheel.setTargetRPM(2810);
		driveTrain.autoWait();
		driveTrain.forwardPID(-9, 127 / 2, true);
		intake_mtr.move_velocity(0);
		driveTrain.tank(-40, -40);
		pros::delay(300);
		intake_mtr.move_velocity(-200);
		pros::delay(400);
		intake_mtr.move_velocity(0);
		driveTrain.swingPID(-7, 127 / 2, false);
		driveTrain.autoWait();
		indexer.shoot(true);
		indexer.shoot(true);
		indexer.shoot(true);
		pros::delay(350);
		intake_mtr.move_velocity(200);
		driveTrain.swingPID(-50, 127 / 2, false);
		driveTrain.autoWaitUntil(1);
		driveTrain.turnPID(-135, 127 / 2);
		driveTrain.autoWait();
		flywheel.setTargetRPM(2450);
		driveTrain.forwardPID(-62, 55, true);
		// driveTrain.followProfileForward(generateProfile(0, -62 / 12, 2, 20.25, 40, 0.0001));
		driveTrain.autoWait();
		driveTrain.turnPID(-47, 127 / 2);
		driveTrain.autoWait();
		indexer.shoot(true);
		indexer.shoot(true);
		indexer.shoot(true);
		pros::delay(350);
		driveTrain.turnPID(-137, 127 / 2);
		driveTrain.autoWait();
		flywheel.setTargetRPM(2550);
		driveTrain.forwardPID(-40, 127 * 0.75, true);
		driveTrain.autoWaitUntil(25);
		driveTrain.setMaxSpeed(127 / 4);
		driveTrain.autoWaitUntil(10);
		driveTrain.setMaxSpeed(127 / 6);
		driveTrain.autoWait();
		driveTrain.turnPID(-85, 127 / 2);
		driveTrain.autoWait();
		indexer.shoot(true);
		indexer.shoot(true);
		indexer.shoot(true);
		pros::delay(350);
		driveTrain.forwardPID(-9, 127 / 2, true);
		driveTrain.autoWait();
		driveTrain.swingPID(-90, 127 / 2, true);
		driveTrain.autoWait();
		flywheel.setTargetRPM(2940);
		driveTrain.forwardPID(-24, 127 / 4, true);
		driveTrain.autoWait();
		driveTrain.turnPID(-89, 127 / 2);
		driveTrain.autoWait();
		indexer.shoot(true);
		indexer.shoot(true);
		indexer.shoot(true);
		pros::delay(350);
		driveTrain.swingPID(-90, 127 / 2, true);
		driveTrain.autoWait();
		driveTrain.forwardPID(-6, 127 / 2, true);
		driveTrain.autoWait();
		intake_mtr.move_velocity(0);
		driveTrain.tank(-30, -30);
		intake_mtr.move_velocity(-200);
		pros::delay(300);
		intake_mtr.move_velocity(200);
		pros::delay(600);
		intake_mtr.move_velocity(0);
		driveTrain.forwardPID(27, 127 / 2, true);
		driveTrain.autoWait();
		driveTrain.turnPID(-180, 127 / 2);
		driveTrain.autoWait();
		driveTrain.forwardPID(-22, 127 / 2, true);
		driveTrain.autoWait();
		driveTrain.tank(-30, -30);
		pros::delay(300);
		intake_mtr.move_velocity(-200);
		pros::delay(300);
		intake_mtr.move_velocity(0);
		pros::delay(100);
		driveTrain.forwardPID(34, 127 / 2, true);
		driveTrain.autoWait();
		driveTrain.swingPID(-135, 127 / 2, false);
		driveTrain.autoWait();
		driveTrain.forwardPID(-26, 127 / 2, true);
		driveTrain.autoWait();
		expansion.setExtended(true);
	}
	driveTrain.endAuto();
	// printf("Auto Ended\n");
}

void opcontrol()
{
	playFreq(0);
	if (disabledCounter > 0)
	{
		driverStarted = true;
	}
	// flywheel.setTargetRPM(3030);
	// driveTrain.swingPID(-10, 127 / 2, false);
	// driveTrain.autoWait();
	// pros::delay(1000);
	// indexer.shoot(true);
	// indexer.shoot(true);
	// indexer.shoot(true);
	// pros::delay(350);

	// goal = 1000;
	// while (goal <= 13000)
	// {
	// 	flywheel.setTargetRPM(goal);
	// 	pros::delay(5000);
	// 	printf("\nKv: %d|%f,%f", goal, flywheel_motor.get_velocity(), flywheel_motor_two.get_velocity());
	// 	goal += 1000;
	// }
	// indexer_mtr.set_brake_mode(MOTOR_BRAKE_COAST);
	// while (true)
	// {
	// 	printf("potentiometer: %i\n", potentiometer.get_value());
	// 	pros::delay(20);
	// }

	double moveSpeed = 0;
	double rotSpeed = 0;

	xPos = 0;
	yPos = 0;
	angle = 0;
	// printf("Generation Started\n");
	// maxAccel = 37.0541918 / mass * 3.281
	// Max m/s = 4.76583968
	// std::vector<SetPoint> profile = generateProfile(0, 3, 1.5, 20.25, 40, 0.0001);

	// printf("Profile Finished\n");
	// double testVoltage = 0;
	// flywheel.setTargetRPM(3000);
	// pros::delay(2000);
	// indexer.shoot(true);
	// autonomous();
	// driveTrain.followProfileForward(generateProfile(0, 1, 4, 20.25, 40, 0.0001));
	// while (true)
	// {
	// 	// printf("dist: %f\n", driveTrain.getEncoders() / 50.277778 / 12);
	// 	pros::delay(20);
	// }

	// 	testVoltage += 0.05;
	// 	driveTrain.tank(testVoltage, testVoltage);
	// 	printf("Voltage: %f, Velocity: %f\n", testVoltage, driveTrain.getVelocity());

	// driveTrain.forwardPID(48, 127 / 2, true);
	// pros::ADIAnalogOut coarse('G');
	// pros::ADIAnalogOut fine('H');

	// Sound sound("smb:d=4,o=5,b=100:16e6,16e6,32p,8e6,16c6,8e6,8g6,8p,8g,8p,8c6,16p,8g,16p,8e,16p,8a,8b,16a#,8a,16g.,16e6,16g6,8a6,16f6,8g6,8e6,16c6,16d6,8b,16p,8c6,16p,8g,16p,8e,16p,8a,8b,16a#,8a,16g.,16e6,16g6,8a6,16f6,8g6,8e6,16c6,16d6,8b,8p,16g6,16f#6,16f6,16d#6,16p,16e6,16p,16g#,16a,16c6,16p,16a,16c6,16d6,8p,16g6,16f#6,16f6,16d#6,16p,16e6,16p,16c7,16p,16c7,16c7,p,16g6,16f#6,16f6,16d#6,16p,16e6,16p,16g#,16a,16c6,16p,16a,16c6,16d6,8p,16d#6,8p,16d6,8p,16c6");
	// pros::Task task([&]
	// 				{

	// 					while (true) {
	// 						for (int i = 0; i < sound.notes.size(); i++)
	// 						{
	// 							Note note = sound.notes.at(i);
	// 							// playFreq(note.hz);
	// 							auto output = HzToCommand(note.hz);
	// 							coarse.set_value(output.first);
	// 							fine.set_value(output.second);
	// 							pros::delay(note.duration);
	// 							// playFreq(0);
	// 							// pros::delay(50);
	// 						}
	// 						// playFreq(0);
	// 						pros::delay(1000);
	// 					} });

	// Sound sound("smb:d=4,o=5,b=100:16e6,16e6,32p,8e6,16c6,8e6,8g6,8p,8g,8p,8c6,16p,8g,16p,8e,16p,8a,8b,16a#,8a,16g.,16e6,16g6,8a6,16f6,8g6,8e6,16c6,16d6,8b,16p,8c6,16p,8g,16p,8e,16p,8a,8b,16a#,8a,16g.,16e6,16g6,8a6,16f6,8g6,8e6,16c6,16d6,8b,8p,16g6,16f#6,16f6,16d#6,16p,16e6,16p,16g#,16a,16c6,16p,16a,16c6,16d6,8p,16g6,16f#6,16f6,16d#6,16p,16e6,16p,16c7,16p,16c7,16c7,p,16g6,16f#6,16f6,16d#6,16p,16e6,16p,16g#,16a,16c6,16p,16a,16c6,16d6,8p,16d#6,8p,16d6,8p,16c6");
	// while (true)
	// {
	// 	for (int i = 0; i < sound.notes.size(); i++)
	// 	{
	// 		Note note = sound.notes.at(i);
	// 		// playFreq(note.hz);
	// 		auto output = HzToCommand(note.hz);
	// 		coarse.set_value(output.first);
	// 		fine.set_value(output.second);
	// 		pros::delay(note.duration);
	// 		// playFreq(0);
	// 		// pros::delay(50);
	// 	}
	// 	// playFreq(0);
	// 	pros::delay(1000);
	// }

	driveTrain.endAuto();
	driveTrain.autoStop();
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
			expansion.setExtended(true);
		}

		flywheel.setTargetRPM(isSpinning ? goal : 0);

		intake_mtr.move_velocity(intakeDir * (isRollerMode ? 0 : 1) * 200);
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