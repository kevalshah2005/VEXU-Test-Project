#include "main.h"
#include "constants.h"
#include <iostream>

#include "subsystems/catapult.h"

enum class AutonModes
{
	SQUARE,
	PATHS,
	NONE,
};

static std::shared_ptr<OdomChassisController> chassis;
static std::shared_ptr<AsyncMotionProfileController> chassisProfileController;
static AutonModes selectedAuton = AutonModes::NONE;
static std::shared_ptr<Catapult> catapult;

static void wind_back_arm();

static inline auto auton_mode_to_string(AutonModes mode) -> std::string
{
	switch (mode)
	{
	case AutonModes::SQUARE:
		return "square";
	case AutonModes::PATHS:
		return "square";
	case AutonModes::NONE:
		return "none";
	}
	// unreachable
	abort();
}

void autonSelectorWatcher()
{
	// this can get mucky if two buttons are pressed at the same time. this does not matter tbh
	const uint8_t buttons = pros::lcd::read_buttons();
	if (buttons == 0)
	{
		return;
	}

	if (buttons & LCD_BTN_LEFT)
	{
		selectedAuton = AutonModes::SQUARE;
	}
	else if (buttons & LCD_BTN_CENTER)
	{
		selectedAuton = AutonModes::PATHS;
	}
	else if (buttons & LCD_BTN_RIGHT)
	{
		selectedAuton = AutonModes::NONE;
	}
	else
	{
		abort();
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Comet Robotics VEX-U!");

	MotorGroup mgroup_l{{constants::FL_PORT, constants::BL_PORT}};
	MotorGroup mgroup_r{{constants::FR_PORT, constants::BR_PORT}};

	mgroup_l.setReversed(constants::LEFT_REVERSED);
	mgroup_r.setReversed(constants::RIGHT_REVERSED);

	catapult = std::make_shared<Catapult>();

	chassis =
		ChassisControllerBuilder()
			.withMotors(mgroup_l, mgroup_r)
			.withDimensions(constants::CHASSIS_GEARSET, {constants::CHASSIS_DIMS, constants::CHASSIS_TPR})
			.withOdometry()
			.buildOdometry();

	chassisProfileController =
		AsyncMotionProfileControllerBuilder()
			.withLimits(constants::PATH_LIMITS)
			.withOutput(chassis)
			.buildMotionProfileController();

	for (const comets::path_plan &plan : constants::PATHS)
	{
		chassisProfileController->generatePath(plan.points, std::string(plan.name));
	}

	pros::Task autonSelectorWatcher_task(autonSelectorWatcher);
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
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
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
void autonomous()
{
	catapult->wind_arm();
	pros::delay(2000);
	catapult->release_arm();

	const auto mode_name = auton_mode_to_string(selectedAuton);
	printf("Starting autonomous routine. (%s)\n", mode_name.c_str());

	switch (selectedAuton)
	{
	case AutonModes::SQUARE:
	{
		double oldMaxVel = chassis->getMaxVelocity();
		chassis->setMaxVelocity(125.0);		 // affects paths
		chassis->driveToPoint({1_ft, 1_ft}); // assume starting position of {0, 0, 0} // TODO: figure out what this does
		for (int i = 0; i < 4; i++)
		{
			chassis->moveDistance(2_ft);
			printf("Finished driving for iter %d\n", i);
			chassis->turnAngle(90_deg);
			printf("Finished turning for iter %d\n", i);
		}
		chassis->setMaxVelocity(oldMaxVel);
	}
	break;
	case AutonModes::PATHS:
	{
		chassisProfileController->setTarget("right_turn");
		chassisProfileController->waitUntilSettled();
		turnAngle(-90_deg);
		chassisProfileController->setTarget("straight");
		chassisProfileController->waitUntilSettled();
		chassisProfileController->setTarget("strafe_right");
		chassisProfileController->waitUntilSettled();
	}
	break;
	case AutonModes::NONE:
	{
	}
	break;
	}

	printf("Done with autonomous routine. (%s)\n", mode_name.c_str());
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
	Controller controller;

	while (true)
	{
		// pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		//                  (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		//                  (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		pros::lcd::print(0, "Battery: %f V / %f cap / %f temp", pros::battery::get_voltage() / 1000.0, pros::battery::get_capacity(), pros::battery::get_temperature());

		const auto state = chassis->getState();
		std::cout << state.x.convert(inch) << " " << state.y.convert(inch) << " " << state.theta.convert(degree) << "\n";

		chassis->getModel()->arcade(
			controller.getAnalog(ControllerAnalog::leftY),
			controller.getAnalog(ControllerAnalog::rightX));

		pros::delay(constants::TELEOP_POLL_TIME);
	}
}

void turnAngle(okapi::QAngle angle)
{
	double oldMaxVel = chassis->getMaxVelocity();
	chassis->setMaxVelocity(oldMaxVel * constants::TURN_VEL_MULT);
	chassis->turnAngle(angle);
	chassis->setMaxVelocity(oldMaxVel);
}

