#include <Wire.h>
#include "pins.h"
#include "MPU6500.h"
#include "AttitudeEstimator.h"
#include "MSP.h"
#include "Motors.h"
#include "PID.h"
#include "Receiver.h"

// Set to 1 to use Betaflight Configurator, 0 for human-readable Serial Monitor
#define USE_BETAFLIGHT_MSP 0

MPU6500 imu;
IMUData data;
AttitudeEstimator attitude;

unsigned long previousTime = 0;

void setup() {
  Serial.begin(115200);

  // Immediately initialize ESCs with 1000us zero throttle so SimonK ESCs arm properly!
  Motors_init();

  // Initialize I2C bus with the custom pins defined in pins.h
  Wire.begin(I2C_SDA, I2C_SCL);

#if !USE_BETAFLIGHT_MSP
  delay(1000);
  Serial.println("Project Falcon FC");
  //Serial.println("--------------------");
#endif

  if (!imu.begin()) {
#if !USE_BETAFLIGHT_MSP
    Serial.println("IMU Initialization Failed");
#endif
    while (1) {
      delay(100); // Prevents Watchdog Timer reset!
    }
  }

  // Serial.println("IMU Initialized Successfully");

  //imu.printConfiguration();

  // ----------------------------------------
  // Gyroscope calibration
  // ----------------------------------------

  imu.calibrateGyro(data);

  // ----------------------------------------
  // Accelerometer calibration
  // ----------------------------------------

  imu.calibrateAccelerometer(data);

  // ----------------------------------------
  // Get a fresh sensor reading
  // ----------------------------------------

  if (imu.readRawData(data)) {
    imu.convertRawData(data);

    imu.applyGyroCalibration(data);
    imu.applyAccelCalibration(data);

    // ----------------------------------------
    // Initialize attitude from calibrated
    // accelerometer orientation
    // ----------------------------------------
#if !USE_BETAFLIGHT_MSP
    delay(3000);
#endif
    attitude.initialize(data);

#if !USE_BETAFLIGHT_MSP
    Serial.println("Attitude estimator initialized.");

    Serial.print("Initial Roll : ");
    Serial.println(attitude.getRoll(), 2);

    Serial.print("Initial Pitch : ");
    Serial.println(attitude.getPitch(), 2);
#endif
  } else {
#if !USE_BETAFLIGHT_MSP
    Serial.println("ERROR: Failed to read IMU for attitude initialization.");
#endif

    while (1) {
      delay(100); // Prevents Watchdog Timer reset!
    }
  }

  previousTime = micros();

  // Initialize Flight Controller Modules
  PID_init();
  Receiver_init();
}


void loop() {
  // Listen for NRF24L01 radio commands from transmitter
  Receiver_update();

  // Always evaluate incoming serial commands from Betaflight
#if USE_BETAFLIGHT_MSP
  MSP_evaluate(attitude, data);
#endif

  if (imu.readRawData(data)) {
    unsigned long currentTime = micros();

    float dt =
      (currentTime - previousTime) / 1000000.0f;

    previousTime = currentTime;

    imu.convertRawData(data);
    imu.applyGyroCalibration(data);
    imu.applyAccelCalibration(data);
    attitude.update(data, dt);

    int outRoll = 0, outPitch = 0, outYaw = 0;
    
    // Only compute PID if connection is active (no failsafe) and throttle is up
    if (!Receiver_hasFailsafe() && Receiver_getThrottle() > 1050) {
        PID_compute(attitude.getRoll(), attitude.getPitch(), data.gz, 
                    Receiver_getRoll(), Receiver_getPitch(), Receiver_getYaw(), 
                    dt, outRoll, outPitch, outYaw);
    } else {
        PID_reset(); // Reset integral windup when motors are off
    }

    // Mix and write to motors
    Motors_write(Receiver_hasFailsafe() ? 1000 : Receiver_getThrottle(), outRoll, outPitch, outYaw);

#if USE_BETAFLIGHT_MSP
    // In MSP mode, we don't print human-readable text, because it would corrupt the binary protocol!
#else
    // Only print to Serial Monitor every 500ms to avoid slowing down the flight loop
    static unsigned long lastPrintTime = 0;
    if (millis() - lastPrintTime >= 500) {
        lastPrintTime = millis();
        Serial.print("Roll: ");
        Serial.print(attitude.getRoll(), 1);
        Serial.print("°  Pitch: ");
        Serial.print(attitude.getPitch(), 1);
        Serial.print("°  Yaw: ");
        Serial.print(attitude.getYaw(), 1);
        Serial.print("° | Thr: ");
        Serial.print(Receiver_getThrottle());
        Serial.print(" | ");
        if (Receiver_hasFailsafe()) {
            Serial.print("[DISARMED / FAILSAFE]");
        } else {
            Serial.print("[ARMED - ACTIVE]");
        }
        Serial.println();
    }
#endif
  }
}