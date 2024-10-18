#include "main.h"
#include "navigation.hpp"

// 3.75
// 15

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {

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
	// declare intake
	pros::Motor intakeRoller(-7, pros::v5::MotorGears::green);
	pros::Motor intakeConveyor(6, pros::v5::MotorGears::green);

	goTo(1.2, 1.2);
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
	// initialize screen
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	// declare controller
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	// declare drivetrain motors
	pros::MotorGroup drivetrainLeft({-1, -11}, pros::v5::MotorGears::green);    // Creates a motor group with forwards ports 9 & 10 and reversed port N/A
	pros::MotorGroup drivetrainRight({10, 20}, pros::v5::MotorGears::green);  // Creates a motor group with forwards ports N/A and reversed ports 19 & 20

	// declare intake motors
	pros::Motor intakeRoller(-7, pros::v5::MotorGears::green);
	pros::Motor intakeConveyor(6, pros::v5::MotorGears::green);

	// declare pneumatics
	pros::adi::Pneumatics mobileGoalPiston('A', false); 

	// test-run autonomous
	autonomous();

	while (true) {
		// Tank control scheme
		int leftSpeed = master.get_analog(ANALOG_LEFT_Y);
		int rightSpeed = master.get_analog(ANALOG_RIGHT_Y);

		drivetrainLeft.move(leftSpeed);
		drivetrainRight.move(rightSpeed);

		// run intake based on controller
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			intakeRoller.move(127);
			intakeConveyor.move(127);
		}

		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			intakeRoller.move(-127);
			intakeConveyor.move(-127);
		}

		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
			intakeRoller.brake();
			intakeConveyor.brake();
		}

		// extend piston based on controller
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      		mobileGoalPiston.extend();
    	}

    	else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      		mobileGoalPiston.retract();
		}
		
		// delay to avoid fried Brian
		pros::delay(20);
	}
}