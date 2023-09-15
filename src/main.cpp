#include "main.h"
#include "constants.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

static std::shared_ptr<ChassisController> chassis;

std::shared_ptr<AsyncMotionProfileController> chassisProfileController;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
	
	MotorGroup mgroup_l {{constants::FL_PORT, constants::BL_PORT}};
	MotorGroup mgroup_r {{constants::FR_PORT, constants::BR_PORT}};

	mgroup_l.setReversed(constants::LEFT_REVERSED);
	mgroup_r.setReversed(constants::RIGHT_REVERSED);

	chassis = 
		ChassisControllerBuilder()
			.withMotors(mgroup_l, mgroup_r)
			.withDimensions(constants::CHASSIS_GEARSET, {constants::CHASSIS_DIMS, constants::CHASSIS_TPR})
			// .withGains({0.0002, 0.0, 0.0}, {0.0005, 0.0, 0.0})  // uncomment this line to enable chassis PID
			.build();

	chassisProfileController =
		AsyncMotionProfileControllerBuilder()
			.withLimits({
				1.0, // Maximum linear velocity of the Chassis in m/s
				2.0, // Maximum linear acceleration of the Chassis in m/s/s
				10.0 // Maximum linear jerk of the Chassis in m/s/s/s
			})
			.withOutput(chassis)
			.buildMotionProfileController();

	chassisProfileController->generatePath(
		{{0_ft, 0_ft, 0_deg}, {3_ft, 3_ft, 90_deg}}, "right_turn");
	chassisProfileController->generatePath(
		{{0_ft, 0_ft, 0_deg}, {3_ft, 0_ft, 0_deg}}, "straight");
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
void autonomous() {
	double oldMaxVel = chassis->getMaxVelocity();
	chassis->setMaxVelocity(constants::AUTO_MAX_VELO);
	printf("Setting max velocity to %f, old max velocity was %f\n", constants::AUTO_MAX_VELO, oldMaxVel);

	// for (int i=0; i < 4; i++) {
	// 	chassis->moveDistance(2_ft);
	// 	printf("Finished driving for iter %d\n", i);
	// 	chassis->turnAngle(90_deg);
	// 	printf("Finished turning for iter %d\n", i);
	// }

	chassisProfileController->setTarget("right_turn");
	chassisProfileController->waitUntilSettled();
	chassisProfileController->setTarget("straight");
	chassisProfileController->waitUntilSettled();

	printf("Done with autonomous routine.\n");

	chassis->setMaxVelocity(oldMaxVel);
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
void opcontrol() {
	// Controller controller;

	// while (true) {
	// 	pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
	// 	                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
	// 	                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);

	// 	chassis->getModel()->arcade(
	// 		controller.getAnalog(ControllerAnalog::leftY),
	// 		controller.getAnalog(ControllerAnalog::rightX)
	// 	);

	// 	pros::delay(10);
	// }

	autonomous();
	
}