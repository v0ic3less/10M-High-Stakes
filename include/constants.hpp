#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

// Motor ports
const int LEFT_MOTOR_FRONT = -18;
const int LEFT_MOTOR_MIDDLE = -20;
const int LEFT_MOTOR_BACK = -11;
const int RIGHT_MOTOR_FRONT = 17;
const int RIGHT_MOTOR_MIDDLE = 14;
const int RIGHT_MOTOR_BACK = 12;
const int INTAKE_PORT = 6;
const int HOOK_PORT = 13;
const int IMU_PORT = 2;
const int DISTANCE_PORT = 9;
const int OPTICAL_PORT = 5;

// Pneumatic ports
const char INTAKE_EXTEND_PORT = 'A';
const char INTAKE_RETRACT_PORT = 'B';
const char CLAMP_PORT = 'C';
const char LIFT_PORT = 'D';
const char DIDDY_PORT = 'E';
// Drivetrain settings
const double TRACK_WIDTH = 13; // in inches
const double DRIVETRAIN_RPM = 360;
const double HORIZONTAL_DRIFT = 2;

// Lateral PID Constants
const double LATERAL_KP = 2;
const double LATERAL_KI = 0;
const double LATERAL_KD = 10;
const double LATERAL_ANTI_WINDUP = 0;
const double LATERAL_SMALL_ERROR_RANGE = 0; // in inches
const int LATERAL_SMALL_ERROR_TIMEOUT = 0; // in milliseconds
const double LATERAL_LARGE_ERROR_RANGE = 0; // in inches
const int LATERAL_LARGE_ERROR_TIMEOUT = 0; // in milliseconds
const double LATERAL_MAX_ACCEL = 0;

// Angular PID Constants
const double ANGULAR_KP = 3;
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
