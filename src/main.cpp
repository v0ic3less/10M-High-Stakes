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

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

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
    // loop forever
    while (true) {
        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
		bool intake_extended = false;

        // move the robot
        chassis.tank(leftY, rightY);


		if (controller.get_digital(DIGITAL_R1)) {
			intake.move_velocity(200); // This is 100 because it's a 100rpm motor
			hook.move_velocity(200);
		}
		else {
			intake.move_velocity(0);
			hook.move_velocity(0);
		}

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
			if (intake_extended){
				intake_extended = false;
				intake_pneumatic_extend.set_value(false);
    			intake_pneumatic_retract.set_value(true);

			} else {
				intake_extended = true;
				intake_pneumatic_extend.set_value(true);
    			intake_pneumatic_retract.set_value(false);
			}
		}

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			//change doinker state
		}
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
			//change segregation state
		}
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
			//change clamp state
		}


        // delay to save resources
        pros::delay(25);
    }
}