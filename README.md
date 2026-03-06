# KittyV1 — Rocket Flight Computer

KittyV1 is an STM32-based actively controlled model rocket. It uses rear-based canard fins driven by a PCA9685 servo controller to stabilize the rocket during flight, with real-time attitude estimation from a quaternion-based Mahony filter.

## Features

- **Quaternion attitude estimation** — Mahony filter on an ICM-42670-P IMU, with adaptive accelerometer trust that automatically reduces correction during high-g boost phases
- **Canard fin control** — 4-channel PCA9685 PWM servo driver for active roll/pitch stabilization via PID control
- **Multi-sensor fusion** — Dual IMU architecture (low-g for precision, high-g for boost phase), barometer, GPS, and magnetometer
- **LoRa telemetry** — Wireless downlink of flight data
- **SD card logging** — Onboard data recording for post-flight analysis
- **3D attitude visualizer** — Browser-based tool using Three.js and the Web Serial API for real-time orientation display, with STEP model import, data recording, and CSV export

## Hardware

| Component | Role |
|---|---|
| STM32 MCU | Main processor |
| ICM-42670-P | Primary 6-axis IMU (16g / 2000 dps) |
| H3LIS331DLTR | High-g 3-axis accelerometer (up to 400g) for boost phase |
| PCA9685 | 16-channel I2C PWM driver for canard servos |
| BME280 | Barometric pressure / altitude |
| SAM-M8Q | GPS receiver |
| Magnetometer | Heading reference |
| RA-02 (SX1278) | LoRa telemetry downlink (SPI, 433 MHz) |
| SD card | Flight data logging |

## Pin Map

See [config.h](KittyV1Main/config.h) for the full pin assignment table. Key connections:

- **I2C** — `PB6` (SCL), `PB7` (SDA) at 400 kHz
- **SPI** — `PA5` (SCK), `PA6` (MISO), `PA7` (MOSI)
- **Canards** — PCA9685 channels 0–3, servo power on channel 15

## Software Architecture

```
KittyV1Main/
├── KittyV1Main.ino   # Setup, main loop, IMU read, Mahony filter
├── config.h          # All pin definitions, constants, and extern declarations
└── globals.cpp       # Global variable storage (single definition point)
```

The main loop runs at **250 Hz**. Each iteration:

1. Reads raw IMU data (accelerometer + gyroscope)
2. Runs the quaternion Mahony filter with adaptive accelerometer gain
3. Extracts Euler angles for telemetry
4. (Future) Computes PID output and drives canard servos

### Attitude Filter

The Mahony filter maintains a unit quaternion avoiding gimbal lock and uses an adaptive proportional gain:

```
Kp_effective = MAHONY_KP / (1 + |accel_magnitude - 1g| * R_ACCEL_SCALE)
```

During powered flight (high g-load), the accelerometer correction is automatically attenuated so the gyroscope dominates. Near 1g (coast/descent), full accelerometer correction prevents drift.

A legacy Euler-angle Kalman filter is preserved in comments for reference/rollback.

## Attitude Visualizer

Open `cube_visualizer.html` in a Chromium-based browser (Chrome, Edge) to get a real-time 3D view of the rocket's orientation over USB serial.

**Features:**
- Connect via Web Serial API (115200 baud)
- Load a `.STEP` CAD model of your rocket (persisted in IndexedDB)
- Real-time roll/pitch/yaw HUD and graph
- Model offset and yaw offset controls for alignment
- Record attitude data and export to CSV
- Custom logo upload

## Getting Started

1. **Flash the firmware** — Open `KittyV1Main/KittyV1Main.ino` in the Arduino IDE (or PlatformIO) with STM32 board support installed
2. **Install libraries** — `ICM42670P`, `LoRa`, `Adafruit_NeoPixel`, `Melopero_SAM_M8Q`, `BME280I2C`
3. **Connect hardware** — Wire sensors per the pin map in `config.h`
4. **Visualize** — Open `cube_visualizer.html` in Chrome, click "Connect Serial", and select the COM port

## License

Copyright © 2026 Luka Gruev
All Rights Reserved.

No copying, redistribution, modification, or commercial use
is permitted without explicit written permission.
