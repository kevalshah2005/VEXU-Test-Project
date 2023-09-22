#include "main.h"
#include "constants.h"
#include "overloaded.h"

static std::shared_ptr<OdomChassisController> chassis;
static std::shared_ptr<AsyncMotionProfileController> chassisProfileController;

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
	for (const auto &path : constants::AUTON_PATH_TARGETS)
	{
		chassisProfileController->generatePath(path.points, std::string(path.name));
	}
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
	const double MAX_VEL = chassis->getMaxVelocity();

	chassis->setState({0_in, 0_in, 0_deg});
	for (const auto &instruction : constants::AUTON_INSTRUCTIONS)
	{
		std::visit(comets::overloaded{
					   [&](comets::turn_tag angle)
					   {
						   chassis->setMaxVelocity(MAX_VEL * constants::TURN_VEL_MULT);
						   chassis->turnAngle(angle);
						   chassis->setMaxVelocity(MAX_VEL);
					   },
					   [&](comets::path_tag path)
					   {
						   chassisProfileController->setTarget(path);
						   chassisProfileController->waitUntilSettled();
					   },
				   },
				   instruction);
	}
	printf("Done with autonomous routine.\n");
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
	const auto old = chassis->getMaxVelocity();
	chassis->setMaxVelocity(old * constants::TURN_VEL_MULT);
	chassis->turnAngle(angle);
	chassis->setMaxVelocity(old);
}
