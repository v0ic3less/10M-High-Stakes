#include "main.h"
#include "lemlib/api.hpp"
#include "pros/misc.h"
#include "constants.hpp"


pros::MotorGroup left_motors({LEFT_MOTOR_FRONT,LEFT_MOTOR_MIDDLE,LEFT_MOTOR_BACK}, pros::MotorGearset::blue); // left motors on ports 1, 2, 3 AND reversed
pros::MotorGroup right_motors({RIGHT_MOTOR_FRONT,RIGHT_MOTOR_MIDDLE,RIGHT_MOTOR_BACK}, pros::MotorGearset::blue); // right motors on ports 4, 5, 6
pros::Motor intake (INTAKE_PORT);
pros::Motor hook (HOOK_PORT);

pros::adi::DigitalOut intake_pneumatic_extend(INTAKE_EXTEND_PORT);  // Port A controls extension
pros::adi::DigitalOut intake_pneumatic_retract(INTAKE_RETRACT_PORT); // Port B controls retraction

pros::adi::DigitalOut clamp_pneumatic(CLAMP_PORT);  // Port A controls extension

lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              TRACK_WIDTH, // 13 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              DRIVETRAIN_RPM, // drivetrain rpm is 360
                              HORIZONTAL_DRIFT // horizontal drift is 2 (for now)
);

pros::Imu imu(IMU_PORT);

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// Lateral PID controller
lemlib::ControllerSettings lateral_controller(LATERAL_KP, 
                                              LATERAL_KI, 
                                              LATERAL_KD, 
                                              LATERAL_ANTI_WINDUP, 
                                              LATERAL_SMALL_ERROR_RANGE, 
                                              LATERAL_SMALL_ERROR_TIMEOUT, 
                                              LATERAL_LARGE_ERROR_RANGE, 
                                              LATERAL_LARGE_ERROR_TIMEOUT, 
                                              LATERAL_MAX_ACCEL);

// Angular PID controller
lemlib::ControllerSettings angular_controller(ANGULAR_KP, 
                                              ANGULAR_KI, 
                                              ANGULAR_KD, 
                                              ANGULAR_ANTI_WINDUP, 
                                              ANGULAR_SMALL_ERROR_RANGE, 
                                              ANGULAR_SMALL_ERROR_TIMEOUT, 
                                              ANGULAR_LARGE_ERROR_RANGE, 
                                              ANGULAR_LARGE_ERROR_TIMEOUT, 
                                              ANGULAR_MAX_ACCEL);
											  
// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(THROTTLE_DEADBAND, 
                                      THROTTLE_MIN_OUTPUT, 
                                      THROTTLE_CURVE_GAIN);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(THROTTLE_DEADBAND, 
                                   THROTTLE_MIN_OUTPUT, 
                                   THROTTLE_CURVE_GAIN);

// create the chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve, 
                        &steer_curve
);

// create the chassis
// lemlib::Chassis chassis(drivetrain, // drivetrain settings
//                         lateral_controller, // lateral PID settings
//                         angular_controller, // angular PID settings
//                         sensors // odometry sensors
// );

pros::Controller controller(pros::E_CONTROLLER_MASTER);

int color = 0;
int auton = 0;
bool intake_extended = false;
bool clamp_extended = false;
bool intaking = false;
// initialize function. Runs on program startup
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

/*
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void change_color() {
    if (color == 0){
        color += 1;
        pros::lcd::print(4, "Color: blue");
    } else if (color == 1){
        color += 1;
        pros::lcd::print(4, "AUTON");
    } else if (color == 2){
        color = 0;
        pros::lcd::print(4, "Color: red");
    }
}
void change_auton_goal() {
    auton += 1;
    pros::lcd::print(5, "Auton: Goal Side");
}
void change_auton_ring() {
    auton = 0;
    pros::lcd::print(5, "Auton: Ring Side");
}
void disabled() {
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
    pros::lcd::register_btn1_cb(change_color);
    pros::lcd::register_btn0_cb(change_auton_goal);
    pros::lcd::register_btn2_cb(change_auton_ring);
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
ASSET(redRingPath1_txt);
ASSET(redRingPath2_txt);
ASSET(redRingPath3_txt);
ASSET(redRingPath4_txt);
ASSET(redRingPath5_txt);
ASSET(redRingPath6_txt);
void autonomous() {
    if (color == 0 && auton == 0){
        intake_extended = true;
        intake_pneumatic_extend.set_value(true);
        intake_pneumatic_retract.set_value(false);   //extend intake
        chassis.setPose(0, 0, 0);         //set pose
        chassis.follow(redRingPath1_txt, 15, 3000); //follow straight to middle 2 ring stack
        chassis.waitUntil(25);                                         //wait until traveled 25in
        intake.move_velocity(200);                                 //move intake
        chassis.waitUntilDone();                                             //wait until first path complete
        pros::delay(500);                                       //wait 500 miliseconds for ring to intake
        chassis.follow(redRingPath2_txt, 15, 3000);  //start lining up for mogo
        hook.move_velocity(100);                                    //start moving hook
        pros::delay(500);                                       //wait 500ms
        hook.move_velocity(0);                                      //stop intake and hook
        intake.move_velocity(0);
        chassis.waitUntilDone();                                              //wait until path followed
        chassis.follow(redRingPath3_txt, 15, 3000, false); //go to goal
        clamp_pneumatic.set_value(true);                                       //open mogo clamp
        chassis.waitUntilDone();                                               //wait until reach mogo
        clamp_pneumatic.set_value(false);                                      //close mogo clamp
        chassis.follow(redRingPath4_txt, 15, 4000);   //go to 2 2 ring stacks in very center
        hook.move_velocity(200);                                     //turn on intake/hook
        intake.move_velocity(200);
        chassis.follow(redRingPath5_txt, 15, 1000, false); //after intaking one ring, back up to get approach on second
        chassis.follow(redRingPath6_txt, 15, 3000);   //go to other ring
        clamp_pneumatic.set_value(true);                                      //release
    }
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
    // loop forever
    while (true) {
        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(leftY, rightY);


		if (controller.get_digital_new_press(DIGITAL_R1)) {
            intaking = !intaking;
			
		}

        if (intaking) {
            intake.move_velocity(200); // This is 100 because it's a 100rpm motor
			hook.move_velocity(200);
        } else {
            intake.move_velocity(0); // This is 100 because it's a 100rpm motor
			hook.move_velocity(0);
        }

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
			if (intake_extended){
				intake_extended = false;
				intake_pneumatic_extend.set_value(false);
    			intake_pneumatic_retract.set_value(true);
                pros::delay(100);

			} else {
				intake_extended = true;
				intake_pneumatic_extend.set_value(true);
    			intake_pneumatic_retract.set_value(false);
                pros::delay(100);
			}
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			//change doinker state
		}
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
			//change segregation state
		}
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
			clamp_extended = !clamp_extended;
            clamp_pneumatic.set_value(clamp_extended);
		}


        // delay to save resources
        pros::delay(25);
    }
}