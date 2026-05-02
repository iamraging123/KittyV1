#include <Wire.h>
#include <Melopero_SAM_M8Q.h>

/* ---------- SAM-M10Q GPS test (Melopero SAM-M8Q library) ----------
 * Library: https://github.com/melopero/Melopero_SAM-M8Q_Arduino_Library
 * Installs via Arduino Library Manager — search "Melopero SAM-M8Q".
 * Pulls in Melopero_UBX_Protocol as a dependency.
 *
 * The SAM-M10Q speaks the same UBX protocol as the SAM-M8Q, so this
 * library drives it fine (NAV-PVT, CFG-PRT, CFG-RATE, CFG-MSG).
 */
#define PIN_I2C_SDA   PB7
#define PIN_I2C_SCL   PB6
#define I2C_FREQ_HZ   400000

/* Default u-blox I2C address. */
#define GPS_ADDR      SAM_M8Q_DEFAULT_I2C_ADDRESS   // 0x42

Melopero_SAM_M8Q gps;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("Power on successful");
  delay(1000);

  Wire.setSCL(PIN_I2C_SCL);
  Wire.setSDA(PIN_I2C_SDA);
  Wire.begin();
  Wire.setClock(I2C_FREQ_HZ);
  Serial.println("I2C Initialized");

  Serial.println("\nSAM-M10Q GPS test (Melopero SAM-M8Q library)");
  Serial.println("--------------------------------------------");

  gps.initI2C(GPS_ADDR, Wire);

  /* Force UBX-only on the I2C (DDC) port. */
  Status stat = gps.setCommunicationToUbxOnly();
  Serial.print("Set UBX-only: "); Serial.println(gps.getStatusDescription(stat));
  if (stat != Status::NoError) {
    Serial.println("Failed to false t9999999S — check wiring, address (0x42), and power.");
    while (1) delay(100);
  }
  gps.waitForAcknowledge(CFG_CLASS, CFG_PRT);

  /* 5 Hz nav solution: 200 ms measurement, 1 meas / solution. */
  stat = gps.setMeasurementFrequency(200, 1);
  Serial.print("Set rate 5 Hz: "); Serial.println(gps.getStatusDescription(stat));
  gps.waitForAcknowledge(CFG_CLASS, CFG_RATE);

  /* Emit NAV-PVT every navigation solution. */
  stat = gps.setMessageSendRate(NAV_CLASS, NAV_PVT, 1);
  Serial.print("Enable NAV-PVT: "); Serial.println(gps.getStatusDescription(stat));
  gps.waitForAcknowledge(CFG_CLASS, CFG_MSG);

  Serial.println("\n--- Entering loop ---");
}

void loop() {
  Status stat = gps.updatePVT();
  if (stat != Status::NoError) {
    Serial.print("updatePVT: "); Serial.println(gps.getStatusDescription(stat));
    delay(200);
    return;
  }

  /* u-blox scales: lat/lon 1e-7 deg, height 1 mm. */
  const float  lat_deg = gps.pvtData.latitude  * 1e-7f;
  const float  lon_deg = gps.pvtData.longitude * 1e-7f;
  const float  alt_m   = gps.pvtData.height    * 1e-3f;
  const float  hMSL_m  = gps.pvtData.hMSL      * 1e-3f;

  Serial.print("Fix=");  Serial.print(getGNSSFixType(gps.pvtData.fixType));
  Serial.print("  Sats="); Serial.print(gps.pvtData.numberOfSatellites);
  Serial.print("  Lat="); Serial.print(lat_deg, 7);
  Serial.print("  Lon="); Serial.print(lon_deg, 7);
  Serial.print("  Alt="); Serial.print(alt_m, 2);  Serial.print(" m");
  Serial.print("  hMSL=");Serial.print(hMSL_m, 2); Serial.println(" m");

  delay(200);
}
