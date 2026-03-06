/****************************************************
 * globals.cpp
 *
 * Allocates storage for every shared global variable
 * declared as "extern" in config.h.  Each variable is
 * defined exactly once here so the linker can resolve
 * references from KittyV1Main.ino and any future .cpp
 * translation units.
 *
 * Do NOT #include this file — the Arduino build system
 * compiles it automatically and links it in.
 ****************************************************/

#include "Arduino.h"
#include "Wire.h"
#include "ICM42670P.h"
#include "config.h"

/****************************************************
 * ============ CONTROL LOOP TIMING ================
 ****************************************************/

float calibration_samples = 100;      // Number of samples for gyro bias calibration

unsigned long nextLoopTime = 0;       // Scheduled start of next loop iteration (us)
unsigned long lastLoopTime = 0;       // Timestamp of previous loop start (us)
float dt = 0.00000f;                  // Measured delta-time between loops (s)
unsigned long loopCounter = 0;        // Running count of loop iterations

/****************************************************
 * ================ PID GAINS =====================
 ****************************************************/

float KP_ROLL = 0.8f;                // Roll proportional gain
float KI_ROLL = 0.0f;                // Roll integral gain
float KD_ROLL = 0.02f;               // Roll derivative gain

/****************************************************
 * =========== SENSOR RAW DATA =====================
 ****************************************************/

/* --- Low-Range IMU (generic, sensor board) --- */
float imu_low_accel_x_g  = 0.0f;     // X-axis acceleration (g)
float imu_low_accel_y_g  = 0.0f;     // Y-axis acceleration (g)
float imu_low_accel_z_g  = 0.0f;     // Z-axis acceleration (g)
float imu_low_gyro_x_dps = 0.0f;     // X-axis angular rate (deg/s)
float imu_low_gyro_y_dps = 0.0f;     // Y-axis angular rate (deg/s)
float imu_low_gyro_z_dps = 0.0f;     // Z-axis angular rate (deg/s)
float imu_low_temp_c     = 0.0f;     // Die temperature (C)

/* --- High-Range IMU (generic, sensor board) --- */
float imu_high_accel_x_g  = 0.0f;    // X-axis acceleration (g)
float imu_high_accel_y_g  = 0.0f;    // Y-axis acceleration (g)
float imu_high_accel_z_g  = 0.0f;    // Z-axis acceleration (g)
float imu_high_gyro_x_dps = 0.0f;    // X-axis angular rate (deg/s)
float imu_high_gyro_y_dps = 0.0f;    // Y-axis angular rate (deg/s)
float imu_high_gyro_z_dps = 0.0f;    // Z-axis angular rate (deg/s)
float imu_high_temp_c     = 0.0f;    // Die temperature (C)

/* --- Main Board ICM-42670-P --- */
float main_accel_freq  = 350.0f;     // Accelerometer ODR (Hz)
float main_accel_range = 16.0f;      // Accelerometer full-scale (g)
ICM42670 MainIMU(Wire, 0);           // ICM-42670-P driver on Wire bus, addr 0
float main_gyro_freq   = 350.0f;     // Gyroscope ODR (Hz)
float main_gyro_range  = 2000.0f;    // Gyroscope full-scale (dps)
float main_accel_x_g   = 0.0f;      // X-axis acceleration (g)
float main_accel_y_g   = 0.0f;      // Y-axis acceleration (g)
float main_accel_z_g   = 0.0f;      // Z-axis acceleration (g)
float main_gyro_x_dps  = 0.0f;      // X-axis angular rate (deg/s)
float main_gyro_y_dps  = 0.0f;      // Y-axis angular rate (deg/s)
float main_gyro_z_dps  = 0.0f;      // Z-axis angular rate (deg/s)
float main_temp_c      = 0.0f;      // Die temperature (C)
float main_gyro_x_bias = 0.0f;      // X-axis gyro bias correction (deg/s)
float main_gyro_y_bias = 0.0f;      // Y-axis gyro bias correction (deg/s)
float main_gyro_z_bias = 0.0f;      // Z-axis gyro bias correction (deg/s)

float main_gyro_sensitivity_dps_per_lsb = 16.5f; // Gyro sensitivity (deg/s per LSB, set by range) get this and accel sensitivity from the datasheet https://www.hxkchips.com/static/upload/file/20250710/1752135483439094.pdf
float main_accel_sensitivity_g_per_lsb = 2048.0f; // Accel sensitivity (g per LSB, set by range)



/* --- Low-Range IMU Sensor Board --- */
float sensor_accel_x_g   = 0.0f;    // X-axis acceleration (g)
float sensor_accel_y_g   = 0.0f;    // Y-axis acceleration (g)
float sensor_accel_z_g   = 0.0f;    // Z-axis acceleration (g)
float sensor_gyro_x_dps  = 0.0f;    // X-axis angular rate (deg/s)
float sensor_gyro_y_dps  = 0.0f;    // Y-axis angular rate (deg/s)
float sensor_gyro_z_dps  = 0.0f;    // Z-axis angular rate (deg/s)
float sensor_gyro_x_bias = 0.0f;    // X-axis gyro bias correction (deg/s)
float sensor_gyro_y_bias = 0.0f;    // Y-axis gyro bias correction (deg/s)
float sensor_gyro_z_bias = 0.0f;    // Z-axis gyro bias correction (deg/s)

/* --- High-Range IMU Sensor Board (boost phase) --- */
float High_sensor_accel_x_g = 0.0f; // X-axis high-g acceleration (g)
float High_sensor_accel_y_g = 0.0f; // Y-axis high-g acceleration (g)
float High_sensor_accel_z_g = 0.0f; // Z-axis high-g acceleration (g)

/* --- Magnetometer / Compass --- */
float mag_x_uT           = 0.0f;    // X-axis magnetic field (uT)
float mag_y_uT           = 0.0f;    // Y-axis magnetic field (uT)
float mag_z_uT           = 0.0f;    // Z-axis magnetic field (uT)
float mag_declination_deg = 2.5f;    // Local magnetic declination (deg)

/* --- Barometer --- */
float baro_pressure_pa = 0.0f;      // Atmospheric pressure (Pa)
float baro_temp_c      = 0.0f;      // Barometer temperature (C)
float baro_altitude_m  = 0.0f;      // Pressure-derived altitude (m)

/* --- GPS --- */
double  gps_lat_deg  = 0.0;         // Raw GPS latitude (deg)
double  gps_lon_deg  = 0.0;         // Raw GPS longitude (deg)
float   gps_alt_m    = 0.0f;        // Raw GPS altitude (m)
uint8_t gps_fix_type = 0;           // Fix type (0=none, 2=2D, 3=3D)

/****************************************************
 * =============== FLIGHT STATE ====================
 ****************************************************/

/* --- Attitude quaternion (body -> inertial) --- */
float q_w = 1.0f;                   // Scalar component (starts at identity)
float q_x = 0.0f;                   // X vector component
float q_y = 0.0f;                   // Y vector component
float q_z = 0.0f;                   // Z vector component

/* --- Fused angular rates (body frame) --- */
float gyro_x_dps = 0.0f;            // Roll rate (deg/s)
float gyro_y_dps = 0.0f;            // Pitch rate (deg/s)
float gyro_z_dps = 0.0f;            // Yaw rate (deg/s)

/* --- Fused linear acceleration (body frame) --- */
float accel_x_g = 0.0f;             // Forward acceleration (g)
float accel_y_g = 0.0f;             // Lateral acceleration (g)
float accel_z_g = 0.0f;             // Vertical acceleration (g)

/* --- Euler angles (derived from quaternion, debug) --- */
float roll_rad  = 0.0f;             // Roll angle (rad)
float pitch_rad = 0.0f;             // Pitch angle (rad)
float yaw_rad   = 0.0f;             // Yaw / heading (rad)

/* --- Navigation --- */
float  altitude_m           = 0.0f;  // Best-estimate altitude (m)
float  vertical_velocity_mps = 0.0f; // Vertical velocity (m/s, positive up)
double gps_lat              = 0.0;   // Filtered latitude (deg)
double gps_lon              = 0.0;   // Filtered longitude (deg)
float  gps_alt              = 0.0f;  // Filtered altitude (m)

/* --- Roll PID state --- */
float roll_error      = 0.0f;       // Current roll error (rad)
float roll_integrator = 0.0f;       // Accumulated roll integral (rad*s)
float roll_derivative = 0.0f;       // Roll error rate of change (rad/s)
float roll_cmd_deg    = 0.0f;       // Canard deflection command (deg)

/****************************************************
 * ================ SETPOINTS =====================
 ****************************************************/

float roll_setpoint_rad  = 0.0f;    // Desired roll angle (rad)
float pitch_setpoint_rad = 0.0f;    // Desired pitch angle (rad)
float yaw_setpoint_rad   = 0.0f;    // Desired yaw angle (rad)

/****************************************************
 * =========== ATTITUDE FILTER (MAHONY) ============
 ****************************************************/

/* Euler angles extracted from quaternion (deg, for telemetry only) */
float mahony_roll  = 0.0f;          // Roll estimate (deg)
float mahony_pitch = 0.0f;          // Pitch estimate (deg)
float mahony_yaw   = 0.0f;          // Yaw estimate (deg)

/* Tunable filter gains */
const float MAHONY_KP     = 2.0f;   // Proportional accel correction (higher = more accel trust)
const float R_ACCEL_SCALE = 50.0f;  // Adaptive scaling — gain reduced when |accel| != 1g
