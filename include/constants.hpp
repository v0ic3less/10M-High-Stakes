#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

// Motor ports
const int LEFT_MOTOR_FRONT = -20;
const int LEFT_MOTOR_MIDDLE = -21;
const int LEFT_MOTOR_BACK = -3;
const int RIGHT_MOTOR_FRONT = 4;
const int RIGHT_MOTOR_MIDDLE = 5;
const int RIGHT_MOTOR_BACK = 6;
const int INTAKE_PORT = 1;
const int HOOK_PORT = -2;
const int IMU_PORT = 10;

// Pneumatic ports
const char INTAKE_EXTEND_PORT = 'A';
const char INTAKE_RETRACT_PORT = 'B';
const char CLAMP_PORT = 'C';

// Drivetrain settings
const double TRACK_WIDTH = 13; // in inches
const double DRIVETRAIN_RPM = 360;
const double HORIZONTAL_DRIFT = 2;

// Lateral PID Constants
const double LATERAL_KP = 10;
const double LATERAL_KI = 0;
const double LATERAL_KD = 3;
const double LATERAL_ANTI_WINDUP = 3;
const double LATERAL_SMALL_ERROR_RANGE = 1; // in inches
const int LATERAL_SMALL_ERROR_TIMEOUT = 100; // in milliseconds
const double LATERAL_LARGE_ERROR_RANGE = 3; // in inches
const int LATERAL_LARGE_ERROR_TIMEOUT = 500; // in milliseconds
const double LATERAL_MAX_ACCEL = 20;

// Angular PID Constants
const double ANGULAR_KP = 2;
const double ANGULAR_KI = 0;
const double ANGULAR_KD = 10;
const double ANGULAR_ANTI_WINDUP = 3;
const double ANGULAR_SMALL_ERROR_RANGE = 1; // in degrees
const int ANGULAR_SMALL_ERROR_TIMEOUT = 100; // in milliseconds
const double ANGULAR_LARGE_ERROR_RANGE = 3; // in degrees
const int ANGULAR_LARGE_ERROR_TIMEOUT = 500; // in milliseconds
const double ANGULAR_MAX_ACCEL = 0;

// Expo curves
const int THROTTLE_DEADBAND = 3;
const int THROTTLE_MIN_OUTPUT = 10;
const double THROTTLE_CURVE_GAIN = 1.019;

#endif
