#pragma once
#include <Arduino.h>
#include <SPI.h>
#include "Config.h"

// Command Packet sent from Ground Station Transmitter to Drone
struct __attribute__((packed)) RadioCommandPacket {
    uint16_t throttle; // 1000 - 2000 us
    int16_t roll;      // target angle in tenths of degree (-300 to +300 = -30.0 to +30.0 deg)
    int16_t pitch;     // target angle in tenths of degree (-300 to +300 = -30.0 to +30.0 deg)
    int16_t yaw;       // target rate in tenths of deg/s (-1500 to +1500)
    uint8_t flags;     // Bit 0: Armed (1=Armed, 0=Disarmed)
                       // Bit 1: Flight Mode (1=Angle, 0=Acro)
    uint8_t checksum;
};

// Telemetry Packet sent from Drone to Ground Station
struct __attribute__((packed)) DroneTelemetryPacket {
    int16_t roll;            // Tenths of degree
    int16_t pitch;           // Tenths of degree
    int16_t yaw;             // Tenths of degree
    int16_t altitudeDm;      // Decimeters (0.1m)
    uint16_t batteryMv;      // Millivolts
    uint8_t gpsSats;
    uint8_t statusFlags;     // Bit 0: Armed, Bit 1: Failsafe, Bit 2: Sensor Health
    uint8_t checksum;
};

class LoRaComm {
public:
    LoRaComm();

    bool begin();
    void update();
    void sendTelemetry(float roll, float pitch, float yaw, float altitude, float vbat, int sats, bool armed, bool failsafe);

    // Extracted control inputs
    float getThrottle() const { return targetThrottle; }
    float getTargetRoll() const { return targetRoll; }
    float getTargetPitch() const { return targetPitch; }
    float getTargetYawRate() const { return targetYawRate; }
    bool isArmedRequested() const { return armedRequested; }
    bool isAngleMode() const { return angleMode; }
    bool isFailsafe() const { return failsafeActive; }

private:
    SPIClass spiLoRa;
    unsigned long lastPacketTime;
    bool failsafeActive;

    float targetThrottle;
    float targetRoll;
    float targetPitch;
    float targetYawRate;
    bool armedRequested;
    bool angleMode;

    void writeRegister(uint8_t reg, uint8_t val);
    uint8_t readRegister(uint8_t reg);
};
