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
#include "SparkFun_u-blox_GNSS_v3.h"
#include "Wire.h"
#include "BME280I2C.h"
#include "EnvironmentCalculations.h"
#include "SD.h"
#include "BME280I2C.h"

#include "config.h"

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

int BaroSetup() {
  if (!bme.begin()) {
    return -1;
  }
  return 0;
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

  if (BaroSetup() != 0) {
    Serial.println("Barometer init failed — altitude KF disabled.");
  } else {
    AltitudeKFInit(50);
  }

  Serial.println("-------- Starting Program --------");
  nextLoopTime = micros();
  lastLoopTime = micros();
}

/****************************************************
 * ===================== LOOP =======================
 ****************************************************/

void loop() {

  /* ---------- dt calculation ---------- */
  unsigned long loopStart = micros();
  dt = (loopStart - lastLoopTime) * 1e-6f;
  lastLoopTime = loopStart;

  IMUread();
  IMUMahony(main_accel_x_g, main_accel_y_g, main_accel_z_g, main_gyro_x_dps, main_gyro_y_dps, main_gyro_z_dps, dt);

  /* ---------- Altitude KF ---------- */
  if (altitude_kf_initialized) {
    /* Predict every loop using world-frame vertical accel from current attitude. */
    a_z_world_mps2 = BodyAccelToWorldVerticalMps2();
    AltitudeKFPredict(a_z_world_mps2, dt);

    /* Correct with baro at BARO_UPDATE_INTERVAL_US cadence (BME280 can't keep up with 250 Hz). */
    if (loopStart - baro_last_read_us >= BARO_UPDATE_INTERVAL_US) {
      BaroRead();
      if (!isnan(baro_pressure_pa) && baro_pressure_pa > 10000.0f) {
        baro_altitude_m = PressureToAltitude(baro_pressure_pa, P0_pa);
        AltitudeKFUpdate(baro_altitude_m, altitude_kf_R_baro);
      }
      baro_last_read_us = loopStart;
    }

    /* FUTURE — when GPS is wired up, call on each fresh fix:
       if (gps_new_sample && gps_fix_type >= 3) {
         AltitudeKFUpdate(gps_alt_m, altitude_kf_R_gps);
       }
    */
  }

  if (loopCounter % SERIAL_PRINT_DIVIDER == 0) {
    /* Format: roll pitch yaw altitude_m vertical_velocity_mps */
    Serial.print(mahony_roll, 2);
    Serial.print(' ');
    Serial.print(mahony_pitch, 2);
    Serial.print(' ');
    Serial.print(mahony_yaw, 2);
    Serial.print(' ');
    Serial.print(altitude_m, 2);
    Serial.print(' ');
    Serial.println(vertical_velocity_mps, 2);
  }
  loopCounter++;



  unsigned long elapsed = micros() - loopStart;
  if (elapsed < CONTROL_LOOP_PERIOD_US) {
    delayMicroseconds(CONTROL_LOOP_PERIOD_US - elapsed);
  }

}




void IMUMahony(
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
  mahony_roll  = atan2f(2.0f * (q_w * q_x + q_y * q_z),
                        1.0f - 2.0f * (q_x * q_x + q_y * q_y)) * (180.0f / PI);

  float sinp   = 2.0f * (q_w * q_y - q_z * q_x);
  if (sinp >  1.0f) sinp =  1.0f;      /* clamp to avoid NaN from float rounding */
  if (sinp < -1.0f) sinp = -1.0f;
  mahony_pitch = asinf(sinp) * (180.0f / PI);

  mahony_yaw   = atan2f(2.0f * (q_w * q_z + q_x * q_y),
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

void BaroRead() {

  float temp(NAN), hum(NAN), pres(NAN);

  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);

  bme.read(pres, temp, hum, tempUnit, presUnit);

  baro_pressure_pa = pres;
  baro_temp_c = temp;
  baro_humidity_percent = hum;
}

/****************************************************
 * ========= ALTITUDE KALMAN FILTER (1D) ============
 * State x = [altitude_m, vertical_velocity_mps]^T
 * Both baro and GPS feed AltitudeKFUpdate(z, R) with
 * the same H = [1, 0] measurement model.
 ****************************************************/

/* Inverse barometric formula — altitude AGL relative to pad pressure P0. */
float PressureToAltitude(float P_pa, float P0_ref_pa) {
  return 44330.0f * (1.0f - powf(P_pa / P0_ref_pa, 0.190263f));
}

/* Project body-frame accel onto world-up, subtract 1 g, return m/s^2.
   The (vx,vy,vz) triple is world-up expressed in body frame — same as in Mahony. */
float BodyAccelToWorldVerticalMps2() {
  float vx = 2.0f * (q_x * q_z - q_w * q_y);
  float vy = 2.0f * (q_w * q_x + q_y * q_z);
  float vz = q_w * q_w - q_x * q_x - q_y * q_y + q_z * q_z;

  float a_up_g = vx * main_accel_x_g + vy * main_accel_y_g + vz * main_accel_z_g;
  return (a_up_g - 1.0f) * GRAVITY_MPS2;
}

/* Capture pad pressure and zero the state. Call once on the pad when armed. */
void AltitudeKFInit(int n_samples) {
  float pressure_sum = 0.0f;
  int   got = 0;
  for (int i = 0; i < n_samples; i++) {
    BaroRead();
    if (!isnan(baro_pressure_pa) && baro_pressure_pa > 10000.0f) {
      pressure_sum += baro_pressure_pa;
      got++;
    }
    delay(20);
  }
  if (got > 0) {
    P0_pa = pressure_sum / (float)got;
  }

  altitude_m              = 0.0f;
  vertical_velocity_mps   = 0.0f;
  altitude_kf_P00         = 1.0f;
  altitude_kf_P01         = 0.0f;
  altitude_kf_P11         = 1.0f;
  altitude_kf_initialized = true;
  baro_last_read_us       = micros();

  Serial.print("Altitude KF initialized. P0 = ");
  Serial.print(P0_pa, 2);
  Serial.println(" Pa");
}

/* Propagate state by dt with accel input u. Q = B*sigma_a^2*B^T, B=[0.5*dt^2, dt]^T. */
void AltitudeKFPredict(float u_mps2, float dt) {
  altitude_m            += vertical_velocity_mps * dt + 0.5f * u_mps2 * dt * dt;
  vertical_velocity_mps += u_mps2 * dt;

  float dt2 = dt * dt;
  float dt3 = dt2 * dt;
  float dt4 = dt2 * dt2;
  float sa2 = altitude_kf_sigma_a * altitude_kf_sigma_a;

  /* P = F*P*F^T + Q with F=[[1,dt],[0,1]] */
  float P00 = altitude_kf_P00 + 2.0f * dt * altitude_kf_P01 + dt2 * altitude_kf_P11 + 0.25f * dt4 * sa2;
  float P01 = altitude_kf_P01 + dt * altitude_kf_P11 + 0.5f * dt3 * sa2;
  float P11 = altitude_kf_P11 + dt2 * sa2;

  altitude_kf_P00 = P00;
  altitude_kf_P01 = P01;
  altitude_kf_P11 = P11;
}

/* Scalar altitude measurement update. Generic — used by baro now, GPS later. */
void AltitudeKFUpdate(float z_meas_m, float R_meas) {

  float y = z_meas_m - altitude_m;
  float S = altitude_kf_P00 + R_meas;
  float inv_S = 1.0f / S;

  float K0 = altitude_kf_P00 * inv_S;
  float K1 = altitude_kf_P01 * inv_S;

  altitude_m            += K0 * y;
  vertical_velocity_mps += K1 * y;

  float P00 = altitude_kf_P00 - K0 * altitude_kf_P00;
  float P01 = altitude_kf_P01 - K0 * altitude_kf_P01;
  float P11 = altitude_kf_P11 - K1 * altitude_kf_P01;

  altitude_kf_P00 = P00;
  altitude_kf_P01 = P01;
  altitude_kf_P11 = P11;
}