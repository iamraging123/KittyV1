/****************************************************
 * ROCKET FLIGHT CONTROL – BASE STRUCTURE
 * Hardware:
 *  - STM32 MCU
 *  - PCA9685 (I2C) for canards
 *  - Dual IMUs (low-g + high-g)
 *  - GPS, Barometer, Compass
 *  - LoRa telemetry
 ****************************************************/

#include "SPI.h"
#include "LoRa.h"
#include "ICM42670P.h"
#include "Adafruit_NeoPixel.h"
#include "Melopero_SAM_M8Q.h"
#include "Wire.h"
#include "BME280I2C.h"
#include "EnvironmentCalculations.h"
#include "SD.h"

#include "config.h"

float calibration_samples = 100;

/****************************************************
 * ===================== SETUP ======================
 ****************************************************/
int MainIMUSetup() {

  while(!Serial) {}

  int MainIMUStatus = MainIMU.begin();

  if (MainIMUStatus != 0) {
    return MainIMUStatus;
  } else {

  // Accel ODR = 100 Hz and Full Scale Range = 16G
  MainIMU.startAccel(main_accel_freq, main_accel_range);
  
  // Gyro ODR = 100 Hz and Full Scale Range = 2000 dps
  MainIMU.startGyro(main_gyro_freq, main_gyro_range);

  return MainIMUStatus;

  }
}

void MainIMUCalibration() {

  Serial.println("Calibrating Main IMU gyroscope biases, please wait...");

  for(int current_sample = 0; current_sample < calibration_samples; current_sample++) {
    inv_imu_sensor_event_t MainIMU_event;
    MainIMU.getDataFromRegisters(MainIMU_event);

    main_gyro_x_bias += MainIMU_event.gyro[0] / main_gyro_sensitivity_dps_per_lsb;
    main_gyro_y_bias += MainIMU_event.gyro[1] / main_gyro_sensitivity_dps_per_lsb;
    main_gyro_z_bias += MainIMU_event.gyro[2] / main_gyro_sensitivity_dps_per_lsb;

    delay(10);
  }

  main_gyro_x_bias /= calibration_samples;
  main_gyro_y_bias /= calibration_samples;
  main_gyro_z_bias /= calibration_samples;
  
  Serial.println("Main IMU gyroscope biases calibrated:");
  Serial.print("X-axis gyro bias: ");
  Serial.print(main_gyro_x_bias, 8);
  Serial.print(" deg/s | Y-axis gyro bias: ");
  Serial.print(main_gyro_y_bias, 8);
  Serial.print(" deg/s | Z-axis gyro bias: ");
  Serial.print(main_gyro_z_bias, 8);
  Serial.println(" deg/s");
}


bool SensorCheck(int MainIMUStatus2, int SensorLowIMUStatus, int SensorHighIMUStatus, int GPSStatus, int BaroStatus, int LoRaStatus, int SDStatus, int PCA9685Status) {
    if (MainIMUStatus2 != 0) {
      Serial.println("Main IMU initialization failed!");
      return false;
    }
    if (SensorLowIMUStatus != 0) {
      Serial.println("Sensor Low IMU initialization failed!");
      return false;
    }
    if (SensorHighIMUStatus != 0) {
      Serial.println("Sensor High IMU initialization failed!");
      return false;
    }
    if (GPSStatus != 0) {
      Serial.println("GPS initialization failed!");
      return false;
    }
    if (BaroStatus != 0) {
      Serial.println("Barometer initialization failed!");
      return false;
    }
    if (LoRaStatus != 0) {
      Serial.println("LoRa initialization failed!");
      return false;
    }
    if (SDStatus != 0) {
      Serial.println("SD Card initialization failed!");
      return false;
    }
    if (PCA9685Status != 0) {
      Serial.println("PCA9685 initialization failed!");
      return false;
  }

  Serial.println("All sensors connected");
  return true;

}

void setup() {

  Serial.begin(115200);
  Serial.println("Power on succesfull");
  delay(3000);

  Wire.begin(PIN_I2C_SCL, PIN_I2C_SDA, I2C_FREQ_HZ);
  for(int i = 0; i < 10; i++) {
    Serial.print(".");
    delay(50);
  }
  
  Serial.println(" ");
  Serial.println("I2C Initialized");
  delay(200);

  //check status of all sensors and throw error if any failed
  MainIMUSetup();
  //SensorCheck(MainIMUSetup(), 0, 0, 0, 0, 0, 0, 0);

  Serial.println("running sensor calibrations...");
  delay(100);
  MainIMUCalibration();

  Serial.println("-------- Starting Program --------");
  nextLoopTime = micros();
  lastLoopTime = micros();
}

/****************************************************
 * ===================== LOOP =======================
 ****************************************************/

void loop() {

  if (START_ONLY) {
    while (true) {
      delay(100);
    }
  }


  /* ---------- dt calculation ---------- */
  unsigned long loopStart = micros();
  dt = (loopStart - lastLoopTime) * 1e-6f;
  lastLoopTime = loopStart;

  IMUread();
  IMUKalman(main_accel_x_g, main_accel_y_g, main_accel_z_g, main_gyro_x_dps, main_gyro_y_dps, main_gyro_z_dps, dt);


  if (loopCounter % SERIAL_PRINT_DIVIDER == 0) {
    Serial.print(" ");
    Serial.print(kalman_roll, 2);
    Serial.print(" ");
    Serial.print(kalman_pitch, 2);
    Serial.print(" ");
    Serial.println(kalman_yaw, 2);
  }
  loopCounter++;



  unsigned long elapsed = micros() - loopStart;
  if (elapsed < CONTROL_LOOP_PERIOD_US) {
    delayMicroseconds(CONTROL_LOOP_PERIOD_US - elapsed);
  }

}

void IMUKalman(
    float accelX, float accelY, float accelZ,
    float gyroX,  float gyroY,  float gyroZ,
    float dt
) {
  /* ---- Bias-corrected gyro rates (deg/s → rad/s) ---- */
  float gx = (gyroX - main_gyro_x_bias) * (PI / 180.0f);
  float gy = (gyroY - main_gyro_y_bias) * (PI / 180.0f);
  float gz = (gyroZ - main_gyro_z_bias) * (PI / 180.0f);

  /* ---- ACCEL CORRECTION (Mahony proportional) ---- */
  float accel_mag = sqrtf(accelX * accelX + accelY * accelY + accelZ * accelZ);

  if (accel_mag > 0.01f) {
    /* Normalize measured accel to unit gravity direction */
    float inv_mag = 1.0f / accel_mag;
    float ax = accelX * inv_mag;
    float ay = accelY * inv_mag;
    float az = accelZ * inv_mag;

    /* Estimated gravity direction in body frame from quaternion.
       This is the third row of the body→world rotation matrix,
       giving "world Z (up)" expressed in body coordinates. */
    float vx = 2.0f * (q_x * q_z - q_w * q_y);
    float vy = 2.0f * (q_w * q_x + q_y * q_z);
    float vz = q_w * q_w - q_x * q_x - q_y * q_y + q_z * q_z;

    /* Error = cross(measured, estimated) — the rotation needed to
       align the quaternion's gravity prediction with the real accel */
    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float ez = ax * vy - ay * vx;

    /* Adaptive gain: full correction near 1g, nearly zero during thrust */
    float accel_dev = fabsf(accel_mag - 1.0f);
    float Kp = MAHONY_KP / (1.0f + accel_dev * R_ACCEL_SCALE);

    gx += Kp * ex;
    gy += Kp * ey;
    gz += Kp * ez;
  }

  /* ---- QUATERNION INTEGRATION (first-order) ---- */
  float halfdt = 0.5f * dt;
  float qw = q_w, qx = q_x, qy = q_y, qz = q_z;

  q_w += (-qx * gx - qy * gy - qz * gz) * halfdt;
  q_x += ( qw * gx + qy * gz - qz * gy) * halfdt;
  q_y += ( qw * gy - qx * gz + qz * gx) * halfdt;
  q_z += ( qw * gz + qx * gy - qy * gx) * halfdt;

  /* Normalize to unit quaternion */
  float inv_norm = 1.0f / sqrtf(q_w * q_w + q_x * q_x + q_y * q_y + q_z * q_z);
  q_w *= inv_norm;
  q_x *= inv_norm;
  q_y *= inv_norm;
  q_z *= inv_norm;

  /* ---- EXTRACT EULER ANGLES (degrees, for telemetry/display) ---- */
  kalman_roll  = atan2f(2.0f * (q_w * q_x + q_y * q_z),
                        1.0f - 2.0f * (q_x * q_x + q_y * q_y)) * (180.0f / PI);

  float sinp   = 2.0f * (q_w * q_y - q_z * q_x);
  if (sinp >  1.0f) sinp =  1.0f;      /* clamp to avoid NaN from float rounding */
  if (sinp < -1.0f) sinp = -1.0f;
  kalman_pitch = asinf(sinp) * (180.0f / PI);

  kalman_yaw   = atan2f(2.0f * (q_w * q_z + q_x * q_y),
                        1.0f - 2.0f * (q_y * q_y + q_z * q_z)) * (180.0f / PI);
}

void IMUread() {
  inv_imu_sensor_event_t MainIMU_event;
  MainIMU.getDataFromRegisters(MainIMU_event);

  main_accel_x_g = MainIMU_event.accel[0] / main_accel_sensitivity_g_per_lsb; // Convert from raw to g
  main_accel_y_g = MainIMU_event.accel[1] / main_accel_sensitivity_g_per_lsb; // Convert from raw to g
  main_accel_z_g = MainIMU_event.accel[2] / main_accel_sensitivity_g_per_lsb; // Convert from raw to g

  main_gyro_x_dps = MainIMU_event.gyro[0] / main_gyro_sensitivity_dps_per_lsb; // Convert from raw to deg/s
  main_gyro_y_dps = MainIMU_event.gyro[1] / main_gyro_sensitivity_dps_per_lsb; // Convert from raw to deg/s
  main_gyro_z_dps = MainIMU_event.gyro[2] / main_gyro_sensitivity_dps_per_lsb; // Convert from raw to deg/s

  main_temp_c = MainIMU_event.temperature;
}

/****************************************************
 * PREVIOUS EULER-ANGLE KALMAN FILTER (for rollback)
 *
 * Limitations:
 *  - Gimbal lock at ±90° pitch (atan2 singularity)
 *  - No adaptive accel noise during thrust
 *
 * To restore: uncomment this, comment out the
 * quaternion IMUKalman above, and re-add P_roll,
 * P_pitch, Q, R to config.h / globals.cpp.
 ****************************************************

void IMUKalman_Euler(
    float accelX, float accelY, float accelZ,
    float gyroX,  float gyroY,  float gyroZ,
    float dt
) {
  kalman_roll  += (gyroX - main_gyro_x_bias) * dt;
  kalman_pitch += (gyroY - main_gyro_y_bias) * dt;
  kalman_yaw   += (gyroZ - main_gyro_z_bias) * dt;

  P_roll  += Q;
  P_pitch += Q;

  float roll_acc  = atan2f(accelY, accelZ) * (180.0f / PI);
  float pitch_acc = atan2f(-accelX, sqrtf(accelY * accelY + accelZ * accelZ)) * (180.0f / PI);

  float K_roll  = P_roll  / (P_roll  + R);
  float K_pitch = P_pitch / (P_pitch + R);

  kalman_roll  += K_roll  * (roll_acc  - kalman_roll);
  kalman_pitch += K_pitch * (pitch_acc - kalman_pitch);

  P_roll  = (1.0f - K_roll)  * P_roll;
  P_pitch = (1.0f - K_pitch) * P_pitch;
}

****************************************************/
