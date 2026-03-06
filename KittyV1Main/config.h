#ifndef CONFIG_H
#define CONFIG_H

/****************************************************
 * config.h
 *
 * Central configuration header for the KittyV1
 * rocket flight computer. Contains all hardware pin
 * assignments, compile-time constants, and extern
 * declarations for every shared global variable.
 *
 * Include this file (and only this file) from any
 * .ino or .cpp that needs access to pins, tuning
 * parameters, or flight-state variables.
 * Actual variable storage lives in globals.cpp.
 ****************************************************/

#include "Arduino.h"
#include "ICM42670P.h"

/****************************************************
 * ===================== CONFIG =====================
 ****************************************************/

/* ---------- Debug ---------- */
#define START_ONLY false           // true = halt after setup (board check only)

/* ---------- I2C ---------- */
#define PIN_I2C_SDA PB7           // I2C data line
#define PIN_I2C_SCL PB6           // I2C clock line
#define I2C_FREQ_HZ 400000        // I2C bus speed (400 kHz fast mode)

/* ---------- SPI ---------- */
#define SPI1_SCK  PA5             // SPI clock
#define SPI1_MISO PA6             // SPI master-in slave-out
#define SPI1_MOSI PA7             // SPI master-out slave-in

/* ---------- Main Board Pins ---------- */
#define SWITCH_SENSOR_BOARD   PC9   // Switch input on the sensor board
#define RGB_LEDS_SENSOR_BOARD PC13  // NeoPixel data for sensor board LEDs
#define LED_SENSOR_BOARD      PC15  // Status LED on sensor board

/* ---------- Sensor Board Pins ---------- */
#define SWITCH_MAIN_BOARD       PC6   // Switch input on the main board
#define RGB_LEDS_MAIN_BOARD     PC10  // NeoPixel data for main board LEDs
#define LED_MAIN_BOARD          PA11  // Status LED on main board
#define BATTERY_READ_MAIN_BOARD PC0   // ADC pin for battery voltage sense
#define FiveVolt_READ_MAIN_BOARD PC1  // ADC pin for 5 V rail sense
#define J3_PINOUT_MAIN_BOARD    PC7   // Spare GPIO header J3
#define J4_PINOUT_MAIN_BOARD    PC8   // Spare GPIO header J4
#define RX_MAIN_BOARD           PA1   // UART receive
#define TX_MAIN_BOARD           PA0   // UART transmit

/* ---------- PCA9685 ---------- */
#define PCA9685_ADDR  0x40        // I2C address of servo driver
#define PCA_PWM_FREQ  50          // PWM frequency for servos (Hz)

/* ---------- Canard Channels ---------- */
#define CANARD_0_CH 0             // PCA9685 channel for canard 0
#define CANARD_1_CH 1             // PCA9685 channel for canard 1
#define CANARD_2_CH 2             // PCA9685 channel for canard 2
#define CANARD_3_CH 3             // PCA9685 channel for canard 3

/* ---------- Servo Power ---------- */
#define SERVO_POWER_CH 15        // PCA9685 channel that enables buck converter

/* ---------- Servo Geometry ---------- */
#define SERVO_CENTER_US 1500.0f  // Neutral pulse width (us)
#define SERVO_MIN_US    1000.0f  // Minimum pulse width (us)
#define SERVO_MAX_US    2000.0f  // Maximum pulse width (us)
#define MAX_CANARD_DEG  20.0f    // Max canard deflection (mechanical limit, deg)

/* ---------- Control Loop ---------- */
#define CONTROL_LOOP_HZ        250.0f  // Main loop rate (Hz)
#define CONTROL_LOOP_PERIOD_US ((unsigned long)(1000000.0f / CONTROL_LOOP_HZ))
extern unsigned long nextLoopTime;     // Scheduled start of next loop iteration (us)
extern unsigned long lastLoopTime;     // Timestamp of previous loop start (us)
extern float dt;                       // Measured delta-time between loops (s)

/* ---------- Serial Print Decimation ---------- */
#define SERIAL_PRINT_DIVIDER 3        // Print every Nth loop iteration
extern unsigned long loopCounter;      // Running count of loop iterations

/* ---------- PID GAINS (ROLL) ---------- */
extern float KP_ROLL;                 // Proportional gain
extern float KI_ROLL;                 // Integral gain
extern float KD_ROLL;                 // Derivative gain

/****************************************************
 * =============== SENSOR RAW DATA ==================
 ****************************************************/

/* ==================================================
 * LOW-RANGE IMU (precision, low noise)
 * Generic low-g sensor readings (sensor board)
 * ================================================== */

/* Accelerometer (g) */
extern float imu_low_accel_x_g;       // X-axis acceleration
extern float imu_low_accel_y_g;       // Y-axis acceleration
extern float imu_low_accel_z_g;       // Z-axis acceleration

/* Gyroscope (deg/s) */
extern float imu_low_gyro_x_dps;      // X-axis angular rate
extern float imu_low_gyro_y_dps;      // Y-axis angular rate
extern float imu_low_gyro_z_dps;      // Z-axis angular rate

/* Temperature */
extern float imu_low_temp_c;          // Die temperature (C)


/* ==================================================
 * HIGH-RANGE IMU (boost, high dynamics)
 * Generic high-g sensor readings (sensor board)
 * ================================================== */

/* Accelerometer (g) */
extern float imu_high_accel_x_g;      // X-axis acceleration
extern float imu_high_accel_y_g;      // Y-axis acceleration
extern float imu_high_accel_z_g;      // Z-axis acceleration

/* Gyroscope (deg/s) */
extern float imu_high_gyro_x_dps;     // X-axis angular rate
extern float imu_high_gyro_y_dps;     // Y-axis angular rate
extern float imu_high_gyro_z_dps;     // Z-axis angular rate

/* Temperature */
extern float imu_high_temp_c;         // Die temperature (C)


/* ==================================================
 * Low-Range IMU MAIN BOARD  (ICM-42670-P)
 * ================================================== */

/* IMU configuration */
extern float main_accel_freq;         // Accelerometer output data rate (Hz)
extern float main_accel_range;        // Accelerometer full-scale range (g)
extern ICM42670 MainIMU;              // ICM-42670-P driver instance

extern float main_gyro_freq;          // Gyroscope output data rate (Hz)
extern float main_gyro_range;         // Gyroscope full-scale range (dps)

extern float main_accel_sensitivity_g_per_lsb;   // Accel sensitivity (LSB/g, set by range)
extern float main_gyro_sensitivity_dps_per_lsb;  // Gyro sensitivity (LSB/dps, set by range)

/* Accelerometer (g) */
extern float main_accel_x_g;          // X-axis acceleration
extern float main_accel_y_g;          // Y-axis acceleration
extern float main_accel_z_g;          // Z-axis acceleration

/* Gyroscope (deg/s) */
extern float main_gyro_x_dps;         // X-axis angular rate
extern float main_gyro_y_dps;         // Y-axis angular rate
extern float main_gyro_z_dps;         // Z-axis angular rate

/* Temperature (C) */
extern float main_temp_c;             // Die temperature

/* Gyroscope biases (deg/s) - subtracted to correct drift */
extern float main_gyro_x_bias;        // X-axis gyro bias
extern float main_gyro_y_bias;        // Y-axis gyro bias
extern float main_gyro_z_bias;        // Z-axis gyro bias

/* ==================================================
 * Low-Range IMU SENSOR BOARD
 * ================================================== */

/* Accelerometer (g) */
extern float sensor_accel_x_g;        // X-axis acceleration
extern float sensor_accel_y_g;        // Y-axis acceleration
extern float sensor_accel_z_g;        // Z-axis acceleration

/* Gyroscope (deg/s) */
extern float sensor_gyro_x_dps;       // X-axis angular rate
extern float sensor_gyro_y_dps;       // Y-axis angular rate
extern float sensor_gyro_z_dps;       // Z-axis angular rate

/* Gyroscope biases (deg/s) */
extern float sensor_gyro_x_bias;      // X-axis gyro bias
extern float sensor_gyro_y_bias;      // Y-axis gyro bias
extern float sensor_gyro_z_bias;      // Z-axis gyro bias

/* ==================================================
 * High-Range IMU SENSOR BOARD
 * ================================================== */

/* Accelerometer (g) - high-g rated for boost phase */
extern float High_sensor_accel_x_g;   // X-axis acceleration
extern float High_sensor_accel_y_g;   // Y-axis acceleration
extern float High_sensor_accel_z_g;   // Z-axis acceleration

/* ==================================================
 * MAGNETOMETER / COMPASS
 * ================================================== */

/* Magnetic field (microtesla) */
extern float mag_x_uT;                // X-axis magnetic field
extern float mag_y_uT;                // Y-axis magnetic field
extern float mag_z_uT;                // Z-axis magnetic field

extern float mag_declination_deg;     // Local magnetic declination correction (deg)

/* ==================================================
 * BAROMETER
 * ================================================== */

/* Pressure & temperature */
extern float baro_pressure_pa;        // Atmospheric pressure (Pa)
extern float baro_temp_c;             // Barometer temperature (C)

/* Altitude */
extern float baro_altitude_m;         // Pressure-derived altitude AGL (m)

/* ==================================================
 * GPS
 * ================================================== */

/* Position */
extern double gps_lat_deg;            // Latitude from GPS receiver (deg)
extern double gps_lon_deg;            // Longitude from GPS receiver (deg)
extern float  gps_alt_m;              // Altitude from GPS receiver (m)

/* Fix status */
extern uint8_t gps_fix_type;          // GPS fix type (0=none, 2=2D, 3=3D)

/****************************************************
 * ===================== STATE ======================
 * Fused / filtered state used by the control loop.
 ****************************************************/

/* ---------- Attitude (Quaternion) ---------- */
/* Body -> Inertial frame quaternion  q = [w, x, y, z] */
extern float q_w;                     // Quaternion scalar component
extern float q_x;                     // Quaternion x vector component
extern float q_y;                     // Quaternion y vector component
extern float q_z;                     // Quaternion z vector component

/* ---------- Angular Rates (Body Frame) ---------- */
/* Fused / selected gyro readings fed to control loop */
extern float gyro_x_dps;              // Roll rate (deg/s)
extern float gyro_y_dps;              // Pitch rate (deg/s)
extern float gyro_z_dps;              // Yaw rate (deg/s)

/* ---------- Linear Acceleration (Body Frame) ---------- */
/* Fused / selected accel readings */
extern float accel_x_g;               // Forward acceleration (g)
extern float accel_y_g;               // Lateral acceleration (g)
extern float accel_z_g;               // Vertical acceleration (g)

/* ---------- Euler Angles (derived from quaternion, debug) ---------- */
extern float roll_rad;                // Roll angle (rad)
extern float pitch_rad;               // Pitch angle (rad)
extern float yaw_rad;                 // Yaw / heading angle (rad)

/* ---------- Navigation ---------- */
extern float altitude_m;              // Best-estimate altitude (m)
extern float vertical_velocity_mps;   // Vertical velocity (m/s, positive up)

extern double gps_lat;                // Fused / filtered latitude (deg)
extern double gps_lon;                // Fused / filtered longitude (deg)
extern float  gps_alt;                // Fused / filtered altitude (m)

/* ---------- Control Errors (Body Frame) ---------- */
/* Roll axis PID state; pitch/yaw added later */
extern float roll_error;              // Current roll error (rad)
extern float roll_integrator;         // Accumulated roll integral (rad*s)
extern float roll_derivative;         // Roll error rate of change (rad/s)

/* ---------- Control Output ---------- */
extern float roll_cmd_deg;            // Canard deflection command for roll (deg)

/****************************************************
 * ===================== SETPOINTS ==================
 ****************************************************/

extern float roll_setpoint_rad;       // Desired roll angle (rad)
extern float pitch_setpoint_rad;      // Desired pitch angle (rad)
extern float yaw_setpoint_rad;        // Desired yaw angle (rad)

/****************************************************
 * =========== ATTITUDE FILTER (MAHONY) ============
 * Quaternion-based — no gimbal lock at any pitch.
 * q_w/q_x/q_y/q_z are declared above in the STATE
 * section and used directly by this filter.
 ****************************************************/

/* Euler angles extracted from quaternion (deg, for telemetry only) */
extern float kalman_roll;             // Roll estimate (deg)
extern float kalman_pitch;            // Pitch estimate (deg, full ±180°)
extern float kalman_yaw;              // Yaw estimate (deg, drifts without magnetometer)

/* Tunable filter gains */
extern const float MAHONY_KP;         // Proportional accel correction gain (higher = more accel trust)
extern const float R_ACCEL_SCALE;     // Adaptive scaling — gain is reduced when |accel| deviates from 1g

#endif // CONFIG_H
