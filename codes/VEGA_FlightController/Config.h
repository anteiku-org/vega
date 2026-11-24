#pragma once
#include <Arduino.h>

/*
 * ======================================================================================
 * PROJECT VEGA: High-Performance ESP32 Flight Controller (ESP-FC Architecture)
 * Microcontroller: ESP32 DevKit (36-Pin / 30-Pin Compatible)
 * Dual-Core FreeRTOS Architecture:
 *   - CORE 1: Hard Real-Time Flight Loop (250Hz - 500Hz Gyro, Mahony AHRS, Cascaded PID, ESCs)
 *   - CORE 0: Background Avionics & Comm (LoRa SX1276, GPS NEO-6M, BMP280, QMC5883L, INA219, DHT11, Buzzer)
 * ======================================================================================
 */

// ==========================================
// 1. PINOUT CONFIGURATION (ESP32 36-PIN)
// ==========================================

// --- Shared I2C Bus ---
#define PIN_I2C_SDA         21
#define PIN_I2C_SCL         22
#define I2C_FREQUENCY       400000 // 400kHz Fast I2C

// --- Hardware SPI Bus (LoRa SX1276) ---
#define PIN_LORA_SCK        18
#define PIN_LORA_MISO       19
#define PIN_LORA_MOSI       23
#define PIN_LORA_CS         5
#define PIN_LORA_RST        14
#define PIN_LORA_DIO0       26
#define LORA_FREQUENCY      433E6  // Set to 433E6, 868E6, or 915E6 matching module

// --- Hardware Serial 2 (NEO-6M GPS) ---
#define PIN_GPS_RX          16     // ESP32 RX2 <- GPS TX
#define PIN_GPS_TX          17     // ESP32 TX2 -> GPS RX
#define GPS_BAUD_RATE       9600

// --- Propulsion / Motor PWM Outputs (Quad-X) ---
#define PIN_MOTOR_RR        32     // Motor 1: Rear-Right (CCW)
#define PIN_MOTOR_FR        33     // Motor 2: Front-Right (CW)
#define PIN_MOTOR_RL        25     // Motor 3: Rear-Left (CW)
#define PIN_MOTOR_FL        27     // Motor 4: Front-Left (CCW)

// --- Auxiliaries ---
#define PIN_DHT11           4      // 1-Wire Temperature & Humidity Sensor
#define PIN_BUZZER          13     // Active Buzzer / Warning Beeper

// ==========================================
// 2. I2C DEVICE ADDRESSES
// ==========================================
#define ADDR_MPU6050        0x68   // Primary Gyro & Accelerometer
#define ADDR_QMC5883L       0x0D   // 3-Axis Compass / Magnetometer
#define ADDR_BMP280         0x76   // Barometer (Default 0x76, fallback 0x77)
#define ADDR_INA219         0x40   // Battery Bus Voltage & Current Monitor

// ==========================================
// 3. FLIGHT CONTROL TIMING & RATES
// ==========================================
#define FLIGHT_LOOP_FREQ_HZ 250    // 250Hz = 4000us loop time (Optimal for I2C MPU6050)
#define FLIGHT_LOOP_DT_S    (1.0f / (float)FLIGHT_LOOP_FREQ_HZ)

#define ESC_PWM_FREQ_HZ     250    // 250Hz ESC refresh rate (4ms period)
#define ESC_MIN_PULSE_US    1000   // Minimum throttle pulse (1000us)
#define ESC_MAX_PULSE_US    2000   // Maximum throttle pulse (2000us)
#define ESC_ARM_PULSE_US    1050   // Idle spinning pulse when armed
#define ESC_STOP_PULSE_US   1000   // Cutoff pulse

// ==========================================
// 4. CASCADED PID DEFAULT GAINS
// ==========================================
// Outer Loop: Angle Error (deg) -> Desired Rate (deg/s)
#define ANGLE_KP_ROLL       4.5f
#define ANGLE_KP_PITCH      4.5f
#define MAX_ANGLE_DEG       45.0f  // Max allowed tilt in Angle / Stabilize mode

// Inner Loop: Rate Error (deg/s) -> Motor Pulse Delta (us)
#define RATE_KP_ROLL        1.35f
#define RATE_KI_ROLL        0.04f
#define RATE_KD_ROLL        0.015f

#define RATE_KP_PITCH       1.35f
#define RATE_KI_PITCH       0.04f
#define RATE_KD_PITCH       0.015f

#define RATE_KP_YAW         2.80f
#define RATE_KI_YAW         0.05f
#define RATE_KD_YAW         0.000f

#define PID_INTEGRAL_LIMIT  150.0f // Anti-windup limit (us)
#define MAX_RATE_DPS        300.0f // Max angular rate (deg/sec)

// ==========================================
// 5. SAFETY & FAILSAFE THRESHOLDS
// ==========================================
#define FAILSAFE_TIMEOUT_MS 500    // Cut power if no radio command for 500ms
#define BATTERY_WARN_V      10.5f  // 3S LiPo low voltage alarm threshold (3.5V/cell)
#define BATTERY_CRITICAL_V  9.9f   // 3S LiPo critical threshold (3.3V/cell)
