#ifndef MPU6500_H
#define MPU6500_H

#include <Arduino.h>
#include <Wire.h>

/******************************************************************************
 * MPU6500 Register Map
 ******************************************************************************/

#define MPU6500_ADDRESS 0x68

// Identification
#define WHO_AM_I 0x75

// Power Management
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

// Configuration
#define CONFIG 0x1A
#define SMPLRT_DIV 0x19

// Gyroscope
#define GYRO_CONFIG 0x1B

// Accelerometer
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG2 0x1D

// Temperature
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42

// Accelerometer Outputs
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

// Gyroscope Outputs
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

// Gyroscope Full Scale Selection
#define GYRO_FS_250DPS 0x00
#define GYRO_FS_500DPS 0x08
#define GYRO_FS_1000DPS 0x10
#define GYRO_FS_2000DPS 0x18


//Accelerometer Full Scale Selection
#define ACCEL_FS_2G 0x00
#define ACCEL_FS_4G 0x08
#define ACCEL_FS_8G 0x10
#define ACCEL_FS_16G 0x18

// Digital Low Pass Filter
#define DLPF_BW_184HZ 0x01
#define DLPF_BW_92HZ 0x02
#define DLPF_BW_41HZ 0x03
#define DLPF_BW_20HZ 0x04
#define DLPF_BW_10HZ 0x05
#define DLPF_BW_5HZ 0x06

#define SAMPLE_RATE_1000HZ 0x00
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42

// Conversion constants
#define ACCEL_SENSITIVITY 8192.0f
#define GYRO_SENSITIVITY 65.5f
#define GRAVITY 9.80665f

struct IMUData {
  //Raw
  int16_t ax_raw;
  int16_t ay_raw;
  int16_t az_raw;

  int16_t gx_raw;
  int16_t gy_raw;
  int16_t gz_raw;

  int16_t temperature_raw;

  // Physical
  float ax;
  float ay;
  float az;

  float gx;
  float gy;
  float gz;

  float temperature;
};

class MPU6500 {

public:

  bool begin();

  bool readRawData(IMUData &data);

  int16_t ax;
  int16_t ay;
  int16_t az;

  int16_t gx;
  int16_t gy;
  int16_t gz;
  int16_t readTemperatureRaw();
  void printConfiguration();
  float getTemperatureCelsius(int16_t raw);

  void convertRawData(IMUData &data);
  void calibrateGyro(IMUData &data);
  void calibrateAccelerometer(IMUData &data);
  MPU6500() {
    gyroBiasX = 0.0f;
    gyroBiasY = 0.0f;
    gyroBiasZ = 0.0f;

    accelBiasX = 0.0f;
    accelBiasY = 0.0f;
    accelBiasZ = 0.0f;

    accelScaleX = 1.0f;
    accelScaleY = 1.0f;
    accelScaleZ = 1.0f;
  }
  void applyGyroCalibration(IMUData &data);
  void applyAccelerometerCalibration(IMUData &data);
  void calibrateAccelerometer6Point(IMUData &data);

  void calibrateAccelerometer6Point();
  void applyAccelCalibration(IMUData &data);

private:
  TwoWire *wire;
  uint8_t readRegister(uint8_t reg);

  void writeRegister(uint8_t reg, uint8_t value);
  float gyroBiasX;
  float gyroBiasY;
  float gyroBiasZ;

private:
  float accelBiasX = 0.0f;
  float accelBiasY = 0.0f;
  float accelBiasZ = 0.0f;

  float accelScaleX = 1.0f;
  float accelScaleY = 1.0f;
  float accelScaleZ = 1.0f;
};
#endif