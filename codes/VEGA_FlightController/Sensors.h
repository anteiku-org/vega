#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "Config.h"
#include "FastMath.h"

struct SensorData {
    // MPU6050
    Vector3f gyroRate;   // deg/s (filtered, bias-corrected)
    Vector3f accelG;     // in g
    bool mpuReady;

    // BMP280
    float pressureHpa;
    float altitudeM;
    float temperatureC;
    bool bmpReady;

    // QMC5883L
    float compassHeading;
    bool qmcReady;

    // INA219
    float batteryVoltage;
    bool inaReady;

    // NEO-6M GPS
    bool gpsFix;
    int gpsSatellites;
    float gpsLatitude;
    float gpsLongitude;
    float gpsAltitude;
    float gpsSpeedKmh;

    // DHT11
    float dhtTemperature;
    float dhtHumidity;
    bool dhtReady;
};

class SensorSuite {
public:
    SensorSuite();

    bool beginI2C();
    bool calibrateGyro(int samples = 400);

    // Fast IMU read (Called on Core 1 at 250Hz - takes < 350us)
    void readFastIMU();

    // Slower avionics read (Called on Core 0 at 20Hz - 50Hz)
    void readBarometer();
    void readCompass();
    void readBattery();
    void updateGPS();
    void readDHT11();

    const SensorData& getData() const { return data; }
    SensorData& getDataRef() { return data; }

private:
    SensorData data;

    // Gyro calibration offsets
    float gyroBiasX, gyroBiasY, gyroBiasZ;

    // BMP280 Trimming parameters
    uint16_t dig_T1;
    int16_t dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    float baselinePressure;

    // GPS NMEA buffer
    String nmeaBuffer;
    void parseNMEA(const String& sentence);

    // I2C helper functions
    void writeRegister8(uint8_t devAddr, uint8_t regAddr, uint8_t value);
    uint8_t readRegister8(uint8_t devAddr, uint8_t regAddr);
};
