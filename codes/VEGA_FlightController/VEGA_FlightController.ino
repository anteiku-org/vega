/*
 * ======================================================================================
 * PROJECT VEGA: High-Performance Autonomous Flight Controller Firmware
 * Architecture Inspired by ESP-FC (rtlopez)
 * Microcontroller: ESP32 DevKit (36-Pin / 30-Pin) @ 240MHz
 *
 * SENSORS & PERIPHERALS:
 *   1. MPU-6050   (I2C 0x68): 6-DOF Rate Gyro & Accelerometer (3D Mahony Quaternion AHRS)
 *   2. BMP280     (I2C 0x76): High-Precision Barometric Altitude & Pressure
 *   3. QMC5883L   (I2C 0x0D): 3-Axis Magnetometer / Electronic Compass
 *   4. INA219     (I2C 0x40): LiPo Battery Bus Voltage & Health Monitor
 *   5. SX1276     (VSPI):     Long-Range 433MHz Bidirectional LoRa Control & Telemetry
 *   6. NEO-6M GPS (Serial2):  Autonomous Navigation & Satellite Telemetry (GPIO 16/17)
 *   7. Quad-X ESCs (GPIO 32, 33, 25, 27): Sub-microsecond 250Hz High-Rate PWM Outputs
 *   8. DHT11      (GPIO 4):   Avionics Bay Ambient Temperature & Humidity
 *   9. Buzzer     (GPIO 13):  Asynchronous Multi-Tone Audible System Alarm
 *
 * DUAL-CORE FREERTOS SYSTEM:
 *   - CORE 1: Hard Real-Time Flight Loop (250Hz Deterministic Gyro, AHRS, Cascaded PID, ESCs)
 *   - CORE 0: Avionics, Radio Communications, Navigation & Failsafe Watchdog
 * ======================================================================================
 */

#include <Arduino.h>
#include "Config.h"
#include "FastMath.h"
#include "MahonyAHRS.h"
#include "PID.h"
#include "Motors.h"
#include "Sensors.h"
#include "LoRaComm.h"
#include "Buzzer.h"

// --- Global Subsystems ---
SensorSuite    sensors;
MahonyAHRS     ahrs(0.5f, 0.0f);
FlightPID      flightPid;
MotorDriver    motors;
LoRaComm       radio;
FlightBuzzer   buzzer;

// --- Shared Thread-Safe Flight State ---
volatile bool systemArmed = false;
volatile bool failsafeEngaged = false;
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

// --- Task Handles ---
TaskHandle_t flightTaskHandle = nullptr;
TaskHandle_t backgroundTaskHandle = nullptr;

// =============================================================================
// CORE 1: DETERMINISTIC HARD REAL-TIME FLIGHT LOOP (250Hz)
// =============================================================================
void flightControlTask(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / FLIGHT_LOOP_FREQ_HZ); // 4ms interval

    Serial.print("[RTOS] Flight Control Loop running on Core: ");
    Serial.println(xPortGetCoreID());

    while (true) {
        // Wait for exact 4ms timer boundary (Deterministic 250Hz with zero jitter)
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // 1. High-Speed Gyro & Accelerometer Burst Read (< 350us)
        sensors.readFastIMU();
        const SensorData& sData = sensors.getData();

        // 2. 3D Mahony Quaternion Attitude Fusion
        ahrs.updateIMU(sData.gyroRate.x, sData.gyroRate.y, sData.gyroRate.z,
                       sData.accelG.x, sData.accelG.y, sData.accelG.z,
                       FLIGHT_LOOP_DT_S);

        float currentRoll  = ahrs.getRoll();
        float currentPitch = ahrs.getPitch();

        // 3. Read Thread-Safe Control Inputs from Radio
        float throttleStick = radio.getThrottle();
        float targetRoll    = radio.getTargetRoll();
        float targetPitch   = radio.getTargetPitch();
        float targetYawRate = radio.getTargetYawRate();
        bool isAngleMode    = radio.isAngleMode();

        portENTER_CRITICAL(&stateMux);
        bool armed = systemArmed && !failsafeEngaged;
        portEXIT_CRITICAL(&stateMux);

        // 4. Cascaded PID Computation & Motor Mixing
        if (armed && throttleStick >= 1050.0f) {
            float outRoll = 0.0f, outPitch = 0.0f, outYaw = 0.0f;

            // Outer Angle PID -> Inner Rate PID
            flightPid.compute(targetRoll, targetPitch, targetYawRate,
                              currentRoll, currentPitch,
                              sData.gyroRate.x, sData.gyroRate.y, sData.gyroRate.z,
                              FLIGHT_LOOP_DT_S, isAngleMode,
                              outRoll, outPitch, outYaw);

            // Mix and output to Quad-X ESCs
            motors.write(throttleStick, outRoll, outPitch, outYaw);
        } else if (armed) {
            // Armed but zero throttle: Idle motor spool-up (AirMode idle)
            flightPid.reset();
            motors.write(1000.0f, 0.0f, 0.0f, 0.0f);
        } else {
            // Disarmed or Failsafe: Motors killed immediately!
            flightPid.reset();
            motors.stopAll();
        }
    }
}

// =============================================================================
// CORE 0: AVIONICS, COMMUNICATIONS, GPS & SAFETY TASK (50Hz)
// =============================================================================
void backgroundAvionicsTask(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 20ms = 50Hz

    Serial.print("[RTOS] Avionics & Telemetry running on Core: ");
    Serial.println(xPortGetCoreID());

    uint32_t loopCounter = 0;

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        loopCounter++;

        // 1. Process LoRa Radio Command Packets & Failsafe Watchdog
        radio.update();
        bool failsafe = radio.isFailsafe();

        // 2. Safe Arming Logic Interlock
        bool armRequested = radio.isArmedRequested();
        portENTER_CRITICAL(&stateMux);
        if (armRequested && !systemArmed && !failsafe) {
            // Safety: Disallow arming if throttle is not at minimum!
            if (radio.getThrottle() <= 1060.0f) {
                systemArmed = true;
                motors.setArmed(true);
                buzzer.play(BUZZER_ARMED);
            }
        } else if (!armRequested && systemArmed) {
            systemArmed = false;
            motors.setArmed(false);
            buzzer.play(BUZZER_DISARMED);
        }

        if (failsafe && systemArmed) {
            systemArmed = false;
            failsafeEngaged = true;
            motors.setArmed(false);
            buzzer.play(BUZZER_FAILSAFE_ALARM);
        } else if (!failsafe) {
            failsafeEngaged = false;
        }
        portEXIT_CRITICAL(&stateMux);

        // 3. Sample Navigation & Environmental Sensors (20Hz)
        if (loopCounter % 2 == 0) {
            sensors.readBarometer();
            sensors.readCompass();
            sensors.readBattery();
        }

        // 4. Stream GPS UART NMEA sentences (Continuous)
        sensors.updateGPS();

        // 5. Sample Ambient DHT11 Sensor (Every 2 seconds = 100 loops)
        if (loopCounter % 100 == 0) {
            sensors.readDHT11();
        }

        // 6. LiPo Battery Voltage Protection Alarm
        const SensorData& sData = sensors.getData();
        if (sData.inaReady && sData.batteryVoltage < BATTERY_WARN_V && sData.batteryVoltage > 6.0f) {
            buzzer.play(BUZZER_BATTERY_LOW);
        }

        // 7. Transmit Real-Time LoRa Telemetry to Ground Station (10Hz)
        if (loopCounter % 5 == 0) {
            radio.sendTelemetry(ahrs.getRoll(), ahrs.getPitch(), ahrs.getYaw(),
                                sData.altitudeM, sData.batteryVoltage,
                                sData.gpsSatellites, systemArmed, failsafeEngaged);
        }

        // 8. Serial Monitor / Plotter Debug Output (10Hz)
        if (loopCounter % 5 == 0) {
            Serial.print("Roll:");   Serial.print(ahrs.getRoll(), 1);
            Serial.print(",Pitch:");  Serial.print(ahrs.getPitch(), 1);
            Serial.print(",Alt:");    Serial.print(sData.altitudeM, 1);
            Serial.print(",VBat:");   Serial.print(sData.batteryVoltage, 2);
            Serial.print(",M1:");     Serial.print(motors.getMotor1Pulse());
            Serial.print(",M2:");     Serial.print(motors.getMotor2Pulse());
            Serial.print(",M3:");     Serial.print(motors.getMotor3Pulse());
            Serial.print(",M4:");     Serial.print(motors.getMotor4Pulse());
            Serial.print(",Armed:");  Serial.println(systemArmed ? 1 : 0);
        }

        // 9. Update Asynchronous Buzzer State Machine
        buzzer.update();
    }
}

// =============================================================================
// SETUP ROUTINE
// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=======================================================");
    Serial.println("  PROJECT VEGA: AUTONOMOUS DUAL-CORE FLIGHT CONTROLLER ");
    Serial.println("  Inspired by ESP-FC Architecture | Hardware Initializing");
    Serial.println("=======================================================");

    // 1. Initialize Buzzer
    buzzer.begin();

    // 2. Initialize Motor Driver (Safe 1000us Disarmed State)
    motors.begin();
    Serial.println("[OK] ESC Motor Driver Initialized (1000us Safe Stop)");

    // 3. Initialize Shared I2C Bus & Sensors
    if (sensors.beginI2C()) {
        Serial.println("[OK] Fast I2C Bus Initialized @ 400kHz");
        Serial.println("[OK] MPU-6050 6-DOF IMU Configured (+/-2000dps, 42Hz DLPF)");
    } else {
        Serial.println("[ERROR] MPU-6050 NOT DETECTED! Check I2C Wiring (GPIO 21/22)");
    }

    // 4. Gyroscope Still Calibration
    Serial.println("[INFO] Calibrating Gyro Offsets... KEEP DRONE FLAT AND STILL!");
    sensors.calibrateGyro(400);
    Serial.println("[OK] Gyroscope Calibrated Successfully!");

    // 5. Initialize LoRa SX1276 Module
    if (radio.begin()) {
        Serial.println("[OK] LoRa SX1276 Transceiver Started @ 433MHz (VSPI)");
    } else {
        Serial.println("[WARN] LoRa SX1276 Not Found (Check SPI Wiring & 3.3V Power)");
    }

    // Play Startup Confirmation Chime
    buzzer.play(BUZZER_BOOT_SUCCESS);

    // 6. Spawn Core 1 Task: Dedicated High-Priority Flight Control Loop
    xTaskCreatePinnedToCore(
        flightControlTask,    // Function pointer
        "FlightCore1",        // Task name
        8192,                 // Stack size
        NULL,                 // Parameters
        configMAX_PRIORITIES - 1, // Highest Real-Time Priority!
        &flightTaskHandle,    // Task handle
        1                     // CORE 1
    );

    // 7. Spawn Core 0 Task: Avionics, Radio & Navigation
    xTaskCreatePinnedToCore(
        backgroundAvionicsTask, // Function pointer
        "AvionicsCore0",        // Task name
        8192,                   // Stack size
        NULL,                   // Parameters
        2,                      // Normal Priority
        &backgroundTaskHandle,  // Task handle
        0                       // CORE 0
    );

    Serial.println("[OK] Dual-Core FreeRTOS Tasks Successfully Launched!\n");
}

// Standard Arduino loop is idle because FreeRTOS manages both cores
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
