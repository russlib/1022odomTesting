#include "main.h"
#include "okapi/api.hpp"
#include <string>


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
	pros::lcd::set_text(1, "thanks shawn");

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





	 std::shared_ptr<OdomChassisController> chassis = ChassisControllerBuilder()
     .withMotors(
         1,  // Top left
         -7, // Top right (reversed)
         -14, // Bottom right (reversed)
         15   // Bottom left
     )

     .withGains(

			 {0.0005, 0, 00.00001}, // Distance controller gains 0.00005 works well but it stops and tilts bot
			 {0.0005, 0, 0.0000020},// 000006 works well // Turn controller gains  {0.001, 0, 0.0000020}
			 {0.0001, 0, 00.00001}  // Angle controller gains (helps drive straight)
			 /*

this works mostly

			 {0.001, 0, 0.0001}, // Distance controller gains
			 {0.001, 0, 00.00007},// 000006 works well // Turn controller gains 0.002 0 0.0001
			 {0.001, 0, 00.00001}


			 			// {0.001, 0, 0.0001}, // Distance controller gains
			 			// {0.0008, 0, 0.00005}, // Turn controller gains
			 			// {0.001, 0, 0.0001}  // Angle controller gains (helps drive straight)
			 /*
         {0.001,0, 0.0001}, // Distance controller gains
         {0.0008, 0.00001, 0.00005}, // Turn controller gains
         {0.001, 0, 0.0001}  // Angle controller gains (helps drive straight)
*/
/* best so far
				 {0.001, 0, 0.0001}, // Distance controller gains
         {0.0008, 0, 0.00005}, // Turn controller gains
         {0.001, 0, 0.0001}  // Angle controller gains (helps drive straight)
    */
		 )
     .withSensors(
         ADIEncoder{'E', 'F'}, // left encoder
         ADIEncoder{'C', 'D', true},  // right encoder (reversed)
         ADIEncoder{'A', 'B'}  // middle encoder
     )

     // Green gearset
     // specify the tracking wheels diameter (2.75 in), track (7 in), and TPR (360)
     // specify the middle encoder distance (1 in) and diameter (2.75 in)
     .withDimensions(AbstractMotor::gearset::green, {{2.75_in, 14.0_in, 2.15_in, 2.75_in}, quadEncoderTPR})
     .withOdometry(StateMode::FRAME_TRANSFORMATION, 2_mm, 1_deg)
		 .withClosedLoopControllerTimeUtil(5,
		 5,
	25_ms)


     .buildOdometry();


 	std::shared_ptr<XDriveModel> model = std::static_pointer_cast<XDriveModel>(
 		std::static_pointer_cast<DefaultOdomChassisController>(chassis)->getModel()
 	);


 	Controller controller;
 	ControllerButton runAutoButton(ControllerDigital::X);
 	ControllerButton checkDistanceButton(ControllerDigital::A);
 	ControllerButton resetDistanceButton(ControllerDigital::B);

 	while (true) {
 		model->xArcade(controller.getAnalog(ControllerAnalog::leftX),
 					controller.getAnalog(ControllerAnalog::leftY),
 					controller.getAnalog(ControllerAnalog::rightX));

 		if (runAutoButton.changedToPressed()) {
 			// Drive the robot in a square pattern using closed-loop control


			// set the state to zero


//chassis->setState({2_ft, 10_ft, 0_deg});
chassis ->setMaxVelocity(600);
//chassis->turnAngle(1800_deg);
/*
std::string encoderTextThree1 = std::to_string(ADIEncoder(ADIEncoder{'A', 'B', true}).get());
	pros::lcd::set_text(2, encoderTextThree1);

	chassis->moveDistance(2_ft);
	chassis->turnAngle(180_deg);
	chassis->moveDistance(2_ft);
	chassis->turnAngle(180_deg);
*/













/*
pros::delay(500);
chassis->turnAngle(720_deg);
chassis->turnAngle(-720_deg);
chassis->turnAngle(90_deg);
pros::delay(500);
chassis->turnAngle(30_deg);
chassis->turnAngle(-30_deg);
*/




		//	chassis->setState({2_ft, 10_ft, 0_deg});
			//chassis->driveToPoint({2.5_ft, 10_ft});
			//chassis->driveToPoint({7.5_ft, 10_ft});
			//chassis->driveToPoint({8_ft, 10_ft});
			//pros::delay(100);
       //chassis->moveDistance(8_ft);

			//chassis->turnAngle(3600_deg);

			// turn 45 degrees and drive approximately 1.4 ft
    //chassis->turnToPoint({12_ft, 0_ft});
		//	chassis->moveDistance(0.7_ft);
			//chassis->turnToPoint({setMaxVelocity(double imaxVelocity)0_ft, 12_ft});







chassis->setState({2_ft, 10_ft, 0_deg});
			chassis->driveToPoint({1.5_ft, 10.5_ft});

			//chassis->driveToPoint({2_ft, 10_ft},true);
chassis->turnToPoint({6_ft, 9_ft});
			chassis->driveToPoint({6_ft, 9_ft});



		  //chassis->turnToAngle(0_deg);
			//chassis->driveToPoint({6_ft, 10_ft});
			chassis -> setMaxVelocity(600);
			chassis->turnToPoint({6_ft, 12_ft});
			pros::delay(100);


			chassis->moveDistance(0.7_ft);
			chassis->moveDistance(-0.7_ft);
			pros::delay(100);

			chassis->turnToPoint({10_ft, 10_ft});
			chassis -> driveToPoint({12_ft, 12_ft});
			chassis->turnToPoint({12_ft, 12_ft});
			chassis->moveDistance(0.4_ft);





			// turn approximately 45 degrees to end up at 90 degrees

		  //chassis->turnToPoint({3_ft, 1_ft});
			//chassis->driveToPoint({4_ft, 1_ft});

			//chassis->turnToAngle(90_deg);
			// turn approximately -90 degrees to face {5_ft, 0_ft} which is to the north of the robot


		//	for (int i = 0; i < 2; i++) {
 				//chassis->moveDistance(24_in); // Drive forward 12 inches
 			//	chassis->turnAngle(180_deg);   // Turn in place 90 degrees
 		//	}
 		}

 		pros::delay(10);
 	}

 }

/*
void opcontrol() {

 auto model = std::make_shared<ThreeEncoderXDriveModel>(1,2,3,4,
	std::make_shared<ADIEncoder>(1,2,true),
	std::make_shared<ADIEncoder>(3,4,true),
	std::make_shared<ADIEncoder>(5,6,true),
200,12000                        );
*/


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
    .withOdometry({{2.75_in, 14_in, 1.5_in, 2.75_in}, 360})
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
					//	Controller controller;
						// ControllerButton runAutoButton(ControllerDigital::X);
						// ControllerButton checkDistanceButton(ControllerDigital::A);
						// ControllerButton resetDistanceButton(ControllerDigital::B);

//while (true) {
/*	model->xArcade(controller.getAnalog(ControllerAnalog::rightX),
	               controller.getAnalog(ControllerAnalog::rightY),
	               controller.getAnalog(ControllerAnalog::leftX));
								 */
	/*
	XDriveModel->getModel()-> xArcade((controller.getAnalog(ControllerAnalog::leftY),
	controller.getAnalog(ControllerAnalog::leftX), controller.getAnalog(ControllerAnalog::rightX));
*/


//(std::static_pointer_cast<okapi::XDriveModel>(XDriveModel->getModel()))->xArcade(controller.getAnalog(ControllerAnalog::leftX),
                //controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightX));


								//if (runAutoButton.changedToPressed()) {
								            // Drive the robot in a square pattern using closed-loop control
								           // for (int i = 0; i < 4; i++) {
								                //XDriveModel->moveDistance(12_in); // Drive forward 12 inches
								               // XDriveModel->turnAngle(90_deg);   // Turn in place 90 degrees
								          //  }
								       // }

							//__APCS_32__	if(checkDistanceButton.changedToPressed()){
// aka A controller button





//
//
// todo
// use the better output tech
								//	int boof =
								//	auto goof = std::to_string(boof);
								//	pros::lcd::set_text(1, goof);


							//	}

							//	if(resetDistanceButton.changedToPressed()){
 							//		pros::lcd::clear_line(1);
								//	pros::lcd::clear_line(2);
//


							//	}




	//pros::delay(10);



//}


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
//}
