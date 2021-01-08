#include "main.h"
#include "okapi/api.hpp"
#include <string>


//check out line 93 shawn



using namespace okapi;
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



	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hell !");

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
void autonomous() {}

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
	/*
 auto model = std::make_shared<ThreeEncoderXDriveModel>(1,2,3,4,
	std::make_shared<ADIEncoder>(1,2,true),
	std::make_shared<ADIEncoder>(3,4,true),
	std::make_shared<ADIEncoder>(5,6,true),
200,12000                        );
*/


//I need the same thing as odom chassic controller but for threeEncoderXDriveModel


//ThreeEncoderXDriveModel



/*
	std::shared_ptr<OdomChassisController> XDriveModel =
	        ChassisControllerBuilder()
	            .withMotors(
							        4,  // Top left
							        -13, // Top right (reversed)
							        -14, // Bottom right (reversed)
							        15   // Bottom left
							    )
	            .withGains(
	                  {0.001, 0, 0.0001}, // Distance controller gains
	                  {0.001, 0, 0.0001}, // Turn controller gains
	                  {0.001, 0, 0.0001}  // Angle controller gains (helps drive straight)
									)
	            // Green gearset, 4 in wheel diam, 11.5 in wheel track
							.withDimensions(AbstractMotor::gearset::green, {{4.17_in, 16.875_in}, imev5GreenTPR})
    .withSensors(
//changed from h g
				ADIEncoder{'E','F' ,false}, // left encoder in ADI ports A & B
//changed from f e
			  ADIEncoder{'C', 'D', true},  // right encoder in ADI ports C & D (reversed)

//changed from d c
				ADIEncoder{'A', 'B',false}  // middle encoder in ADI ports E & F
    )
    // specify the tracking wheels diameter (2.75 in), track (7 in), and TPR (360)
    // specify the middle encoder distance (1 in) and diameter (2.75 in)
    .withOdometry({{2.75_in, 7_in, 1.5_in, 2.75_in}, quadEncoderTPR})
    .buildOdometry();

*/


	/*
//chassis Controller
std::shared_ptr<ChassisController>
ThreeEncoderXDriveModel(1,
2,
3,
4,
{"A","B"},
{"C","D"},
{"E","F"},
100,
12000)
*/
/*
std::shared_ptr<ChassisControllerBuilder> XDriveModel =
ChassisControllerBuilder().withMotors(
        1,  // Top left
        -2, // Top right (reversed)
        -3, // Bottom right (reversed)
        4   // Bottom left
    ) .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
    .withSensors(
        ADIEncoder{'A', 'B'}, // Left encoder in ADI ports A & B
        ADIEncoder{'C', 'D', true}  // Right encoder in ADI ports C & D (reversed)
    )
    // Specify the tracking wheels diam (2.75 in), track (7 in), and TPR (360)
    .withOdometry({{2.75_in, 7_in}, quadEncoderTPR})
    .buildOdometry();
		*/
						Controller controller;
						 ControllerButton runAutoButton(ControllerDigital::X);
						 ControllerButton checkDistanceButton(ControllerDigital::A);
						 ControllerButton resetDistanceButton(ControllerDigital::B);

while (true) {
/*	model->xArcade(controller.getAnalog(ControllerAnalog::rightX),
	               controller.getAnalog(ControllerAnalog::rightY),
	               controller.getAnalog(ControllerAnalog::leftX));
								 */
	/*
	XDriveModel->getModel()-> xArcade((controller.getAnalog(ControllerAnalog::leftY),
	controller.getAnalog(ControllerAnalog::leftX), controller.getAnalog(ControllerAnalog::rightX));
*/


(std::static_pointer_cast<okapi::XDriveModel>(XDriveModel->getModel()))->xArcade(controller.getAnalog(ControllerAnalog::leftX),
                controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightX));


								if (runAutoButton.changedToPressed()) {
								            // Drive the robot in a square pattern using closed-loop control
								            for (int i = 0; i < 4; i++) {
								                XDriveModel->moveDistance(12_in); // Drive forward 12 inches
								                XDriveModel->turnAngle(90_deg);   // Turn in place 90 degrees
								            }
								        }

								if(checkDistanceButton.changedToPressed()){
// aka A controller button





//
//
// todo
// use the better output tech
								//	int boof =
								//	auto goof = std::to_string(boof);
								//	pros::lcd::set_text(1, goof);


								}

								if(resetDistanceButton.changedToPressed()){
 									pros::lcd::clear_line(1);
									pros::lcd::clear_line(2);



								}




	pros::delay(10);



}


	/*
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor left_mtr(1);
	pros::Motor right_mtr(2);

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);

		left_mtr = left;
		right_mtr = right;
		pros::delay(20);

	}
	*/
}
