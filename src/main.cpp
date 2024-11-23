#include "main.h"
#include "lemlib/api.hpp"
#include "pros/misc.h"
#include "constants.hpp"
#include "pros/motors.h"


pros::MotorGroup left_motors({LEFT_MOTOR_FRONT,LEFT_MOTOR_MIDDLE,LEFT_MOTOR_BACK}, pros::MotorGearset::blue); // left motors on ports 1, 2, 3 AND reversed
pros::MotorGroup right_motors({RIGHT_MOTOR_FRONT,RIGHT_MOTOR_MIDDLE,RIGHT_MOTOR_BACK}, pros::MotorGearset::blue); // right motors on ports 4, 5, 6
pros::Motor intake (INTAKE_PORT);
pros::Motor hook (HOOK_PORT);

pros::adi::DigitalOut intake_pneumatic_extend(INTAKE_EXTEND_PORT);  // Port A controls extension
pros::adi::DigitalOut intake_pneumatic_retract(INTAKE_RETRACT_PORT); // Port B controls retraction

pros::adi::DigitalOut clamp_pneumatic(CLAMP_PORT);  // Port A controls extension

pros::adi::DigitalOut lift_pneumatic(LIFT_PORT);

pros::adi::DigitalOut diddy(DIDDY_PORT);

pros::Distance distance_sensor(DISTANCE_PORT);

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
bool lift_extended = false;
bool diddy_extended = false;
int auton_to_run = 1;
bool intaking = false;
int leftY_offset = 0;
int rightX_offset = 0;
bool reversed = false;
bool stopWhenRingDetected = false;
bool ringDetected = false;
// initialize function. Runs on program startup
void initialize() {
    
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // Calibrate joystick drift
    leftY_offset = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    rightX_offset = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // Print offsets to confirm calibration
    pros::lcd::print(7, "Left Y Offset: %d", leftY_offset);
    pros::lcd::print(8, "Right X Offset: %d", rightX_offset);

    intake.set_brake_mode (pros::E_MOTOR_BRAKE_HOLD);

    //auton_to_run = 0; //BLUE NEGATIVE 2 RING 1 STAKE
    //auton_to_run = 1; //BLUE POSITIVE 3 RING 1 STAKE
    auton_to_run = 2; //BLUE POSITIVE 4 RING 2 STAKE
    //auton_to_run = 3; //RED NEGATIVE 2 RING 1 STAKE
    //auton_to_run = 4; //RED POSITIVE 3 RING 1 STAKE
    //auton_to_run = 5; //RED POSITIVE 4 RING 2 STAKE
    //auton_to_run = 6; //skills
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
// ASSET(redRingPath1_txt);
// ASSET(redRingPath2_txt);
// ASSET(redRingPath3_txt);
// ASSET(redRingPath4_txt);
// ASSET(redRingPath5_txt);

// ASSET(redRingPath6_txt);
void autonomous() {
    chassis.setPose(0, 0, 0);
    if (auton_to_run == 0){
        chassis.moveToPoint(0, -33, 2000, {.forwards = false}, false);
        intake_pneumatic_extend.set_value(true);
        intake_pneumatic_retract.set_value(false);
        intake.move_velocity(90);
        chassis.moveToPoint(-8, -16.2, 1500, {}, false);
        chassis.turnToHeading(180, 2000, {.maxSpeed = 60}, false);
        intake.brake();
        intake_pneumatic_extend.set_value(false);
        intake_pneumatic_retract.set_value(true);
        chassis.moveToPoint(-15, -2, 2000,{.maxSpeed=100}, false);
        chassis.moveToPoint(-45, 9, 1800, {.forwards = false, .minSpeed=40}, false);
        clamp_pneumatic.set_value(true);
        pros::delay(500);
        intake.move_velocity(400);
        hook.move_velocity(400);
        pros::delay(2000);
        intake.brake();
        hook.brake();
        // clamp_pneumatic.set_value(false);
        pros::delay(200);
        chassis.turnToHeading(200, 1000, {}, false);
        left_motors.move_velocity(100);
        right_motors.move_velocity(100);
        pros::delay(500);
        left_motors.brake();
        right_motors.brake();
    } else if (auton_to_run == 1){
        // chassis.turnToHeading(45, 1000);
        // chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, -55, 1500, {.forwards = false}, false);
        clamp_pneumatic.set_value(true);
        //chassis.turnToHeading(60, 1000, {}, false);
        chassis.moveToPoint(20, -20, 1300);
        pros::delay(500);
        intake.move_velocity(400);
        hook.move_velocity(400);
        pros::delay(3000);
        chassis.setPose(0,0,0);
        intake_pneumatic_extend.set_value(true);
        intake_pneumatic_retract.set_value(false);
        chassis.moveToPoint(-29, -28, 4400, {.maxSpeed=100}, false);
        pros::delay(1000);
        chassis.setPose(0,0,0);
        // intake_pneumatic_extend.set_value(false);
        // intake_pneumatic_retract.set_value(true);
        chassis.turnToHeading(-100, 1000, {}, false);
        chassis.setPose(0,0,0);
        chassis.moveToPoint(0, 20, 1000);
        hook.brake();
        intake.brake();
        clamp_pneumatic.set_value(false);
        intake_pneumatic_extend.set_value(false);
        intake_pneumatic_retract.set_value(true);
        // right_motors.move_velocity(600);
        // left_motors.move_velocity(600);
        // pros::delay(1500);
        // right_motors.brake();
        // left_motors.brake();
    } else if (auton_to_run == 2){
        // chassis.turnToHeading(45, 1000);
        // chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, -55, 1500, {.forwards = false}, false);
        clamp_pneumatic.set_value(true);
        //chassis.turnToHeading(60, 1000, {}, false);
        chassis.moveToPoint(20, -20, 1300);
        pros::delay(500);
        intake.move_velocity(400);
        hook.move_velocity(400);
        pros::delay(2500);
        intake.brake();
        hook.brake();
        chassis.setPose(0,0,0);
        chassis.moveToPoint(-35, -48, 2400, {.minSpeed=72, .earlyExitRange=2});
        pros::delay(1000);
        clamp_pneumatic.set_value(false);
        pros::delay(1400);
        //chassis.moveToPoint(0, -58, 1400, {.forwards=false}, false);
        chassis.moveToPoint(2, -55, 1400, {.forwards=false, .minSpeed=20}, false);
        pros::delay(200);
        clamp_pneumatic.set_value(true);
        chassis.moveToPoint(-20, -52, 1000, {.minSpeed=20, .earlyExitRange=2}, false);
        intake.move_velocity(400);
        hook.move_velocity(400);
        chassis.moveToPoint(-20, -90, 1000, {.minSpeed=20}, false);
        pros::delay(2000);
        chassis.turnToHeading(20, 1000, {}, false);
        right_motors.move_velocity(400);
        left_motors.move_velocity(400);
        pros::delay(1000);
        clamp_pneumatic.set_value(false);
        right_motors.brake();
        left_motors.brake();
    } else if (auton_to_run == 3) {
        // chassis.turnToHeading(45, 1000);
        // chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, -55, 1500, {.forwards = false}, false);
        clamp_pneumatic.set_value(true);
        //chassis.turnToHeading(60, 1000, {}, false);
        chassis.moveToPoint(-20, -20, 1300);
        pros::delay(500);
        intake.move_velocity(400);
        hook.move_velocity(400);
        pros::delay(3000);
        chassis.setPose(0,0,0);
        intake_pneumatic_extend.set_value(true);
        intake_pneumatic_retract.set_value(false);
        chassis.moveToPoint(29, -28, 4400, {.maxSpeed=100}, false);
        pros::delay(1000);
        chassis.setPose(0,0,0);
        // intake_pneumatic_extend.set_value(false);
        // intake_pneumatic_retract.set_value(true);
        chassis.turnToHeading(-100, 1000, {}, false);
        chassis.setPose(0,0,0);
        chassis.moveToPoint(0, 20, 1000);
        hook.brake();
        intake.brake();
        clamp_pneumatic.set_value(false);
        intake_pneumatic_extend.set_value(false);
        intake_pneumatic_retract.set_value(true);
    }

    

    else if (auton_to_run == 6){
        chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, -5, 500, {.forwards=false, .minSpeed=40}, false);
        clamp_pneumatic.set_value(true);
        pros::delay(500);
        hook.move_velocity(400);
        pros::delay(1000);
        hook.move_velocity(0);
        chassis.setPose(0, 0, 0);
        chassis.moveToPoint(-80, 0, 2000, {.forwards=false}, false);
        pros::delay(500);
        clamp_pneumatic.set_value(false);
        chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, 10, 1000);
        chassis.moveToPoint(0, 0, 1000, {.forwards = false});
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
    if (auton_to_run == 2){
        clamp_pneumatic.set_value(true);
        clamp_extended = true;
    }
    while (true) {
        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) - leftY_offset;
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) - rightX_offset;

        //print distance
        pros::lcd::print(4, "Distance %f", distance_sensor.get_distance()); 

        //get distance value and get first 18 digits
        long long distance = distance_sensor.get_distance();
        std::string numStr = std::to_string(distance);
        std::string first18Digits = numStr.substr(0, 18);
        long long first18AsInt = std::stoll(first18Digits);

        // Compare and print on the LCD
        if (first18AsInt < 40) {
            pros::lcd::print(5, "Here %lld", first18AsInt);
            ringDetected = true;
        } else {
            pros::lcd::print(5, "There %lld", first18AsInt);
            ringDetected = false;
        }

        // move the robot arcade
        chassis.arcade(leftY, rightX);
        
        //if ring in front and ring detection toggled on, stop hook
        if (ringDetected && stopWhenRingDetected) {
            pros::delay(35);
            intake.move_velocity(0);
            pros::delay(90);
            hook.move_velocity(-100);
            pros::delay(500);
            hook.move_velocity(0);
            intaking = false;
            reversed = true;
            //stopWhenRingDetected = false;
            controller.clear();
        }

        //if r1 pressed, toggle if intaking or not
		if (controller.get_digital_new_press(DIGITAL_R1)) {
            intaking = !intaking;
		}

        //if r2 pressed, reverse intake
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            intaking = true;
            reversed = true;
            intake.move_velocity(0);
            hook.move_velocity(-200);
        } else {
            reversed = false;
        }

        //if intaking, intake. if intaking and reversed, stop intake but reverse chain
        if (intaking) {
            if (!reversed && !stopWhenRingDetected){
                intake.move_velocity(400); // This is 100 because it's a 100rpm motor
                hook.move_velocity(200);
            } else if (stopWhenRingDetected){
                intake.move_velocity(400);
                hook.move_velocity(100);
            } else {
                intake.move_velocity(-400);
                hook.move_velocity(-80);
            }
        } else {
            intake.move_velocity(0); // This is 100 because it's a 100rpm motor
			hook.move_velocity(0);
        }

        //if B pressed, toggle active intake
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

        //toggle ring detection
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			stopWhenRingDetected = !stopWhenRingDetected;
            if (stopWhenRingDetected){
                controller.print(1, 1, "Ring Detection");
            } else {
                controller.clear();
            }
		}
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
			//change lift 
            lift_extended = !lift_extended;
            lift_pneumatic.set_value(lift_extended);
		}

        // if L1 pressed, toggle clamp
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
			clamp_extended = !clamp_extended;
            clamp_pneumatic.set_value(clamp_extended);
		}

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
			diddy_extended = !diddy_extended;
            diddy.set_value(diddy_extended);
		}


        // delay to save resources
        pros::delay(25);
    }
}