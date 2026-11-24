#include "MPU6500.h"
#include "pins.h"

bool MPU6500::begin() {
  wire = &Wire;

  delay(100);

  uint8_t id = readRegister(WHO_AM_I);

  Serial.print("WHO_AM_I : 0x");
  Serial.println(id, HEX);

  // 0x70 = MPU6500, 0x68 = MPU6050
  if (id != 0x70 && id != 0x68) {
    Serial.println("ERROR: Unexpected WHO_AM_I");

    return false;
  }
  //-----------------------------------------------------
  // Wake up sensor and select PLL as clock source
  //-----------------------------------------------------

  writeRegister(PWR_MGMT_1, 0x01);

  delay(100);

  //-----------------------------------------------------
  // Digital Low Pass Filter
  //-----------------------------------------------------
  writeRegister(CONFIG, DLPF_BW_41HZ);
  //-----------------------------------------------------
  // Sample Rate
  //-----------------------------------------------------
  writeRegister(SMPLRT_DIV, SAMPLE_RATE_1000HZ);
  //-----------------------------------------------------
  // Gyroscope
  //-----------------------------------------------------
  writeRegister(GYRO_CONFIG, GYRO_FS_500DPS);
  //-----------------------------------------------------
  // Accelerometer
  //-----------------------------------------------------
  writeRegister(ACCEL_CONFIG, ACCEL_FS_4G);
  //-----------------------------------------------------
  // Accelerometer Low Pass Filter
  //-----------------------------------------------------
  writeRegister(ACCEL_CONFIG2, DLPF_BW_41HZ);
  delay(100);
  return true;
}

void MPU6500::writeRegister(uint8_t reg, uint8_t value) {
  wire->beginTransmission(MPU6500_ADDRESS);

  wire->write(reg);
  wire->write(value);

  uint8_t status = wire->endTransmission();

  if (status != 0) {
    Serial.print("I2C write failed. Register = 0x");
    Serial.print(reg, HEX);

    Serial.print("  Status = ");
    Serial.println(status);
  }
}

uint8_t MPU6500::readRegister(uint8_t reg) {
  wire->beginTransmission(MPU6500_ADDRESS);

  wire->write(reg);

  uint8_t status = wire->endTransmission(false);

  if (status != 0) {
    Serial.print("I2C register write failed. Status = ");
    Serial.println(status);

    return 0xFF;
  }

  uint8_t received = wire->requestFrom(
    MPU6500_ADDRESS,
    (uint8_t)1);

  if (received != 1) {
    Serial.print("I2C read failed. Register = 0x");
    Serial.println(reg, HEX);

    return 0xFF;
  }

  return wire->read();
}

bool MPU6500::readRawData(IMUData &data) {
  wire->beginTransmission(MPU6500_ADDRESS);

  wire->write(ACCEL_XOUT_H);

  wire->endTransmission(false);

  wire->requestFrom(MPU6500_ADDRESS, 14);

  if (wire->available() < 14)
    return false;

  // -----------------------------
  // Accelerometer RAW
  // -----------------------------
  data.ax_raw = (int16_t)((wire->read() << 8) | wire->read());
  data.ay_raw = (int16_t)((wire->read() << 8) | wire->read());
  data.az_raw = (int16_t)((wire->read() << 8) | wire->read());

  // -----------------------------
  // Temperature RAW
  // -----------------------------
  data.temperature_raw =
    (int16_t)((wire->read() << 8) | wire->read());

  // -----------------------------
  // Gyroscope RAW
  // -----------------------------
  data.gx_raw = (int16_t)((wire->read() << 8) | wire->read());
  data.gy_raw = (int16_t)((wire->read() << 8) | wire->read());
  data.gz_raw = (int16_t)((wire->read() << 8) | wire->read());

  return true;
}

int16_t MPU6500::readTemperatureRaw() {
  uint8_t high = readRegister(TEMP_OUT_H);
  uint8_t low = readRegister(TEMP_OUT_L);

  int16_t temperature =
    ((int16_t)high << 8) | low;

  return temperature;
}

float MPU6500::getTemperatureCelsius(int16_t raw) {
  return ((float)raw / 333.87f) + 21.0f;
}

// Converting Raw Data to Useful value
void MPU6500::convertRawData(IMUData &data) {
  // --------------------------------
  // Accelerometer: raw → m/s²
  // --------------------------------
  data.ax = ((float)data.ax_raw / ACCEL_SENSITIVITY) * GRAVITY;
  data.ay = ((float)data.ay_raw / ACCEL_SENSITIVITY) * GRAVITY;
  data.az = ((float)data.az_raw / ACCEL_SENSITIVITY) * GRAVITY;


  // --------------------------------
  // Gyroscope: raw → °/s
  // --------------------------------
  data.gx = (float)data.gx_raw / GYRO_SENSITIVITY;
  data.gy = (float)data.gy_raw / GYRO_SENSITIVITY;
  data.gz = (float)data.gz_raw / GYRO_SENSITIVITY;


  // --------------------------------
  // Temperature: raw → °C
  // --------------------------------
  data.temperature =
    ((float)data.temperature_raw / 333.87f) + 21.0f;
}
// Calibrating Gyro
void MPU6500::calibrateGyro(IMUData &data) {
  const int samples = 500;

  float sumX = 0.0f;
  float sumY = 0.0f;
  float sumZ = 0.0f;

  Serial.println();
  Serial.println("================================");
  Serial.println("     GYRO CALIBRATION");
  Serial.println("================================");
  Serial.println("Keep the flight controller STILL.");
  Serial.println("Do not touch it...");
  Serial.println();

  delay(2000);

  int successfulSamples = 0;
  
  while (successfulSamples < samples) {
    if (readRawData(data)) {
      convertRawData(data);

      sumX += data.gx;
      sumY += data.gy;
      sumZ += data.gz;
      
      successfulSamples++;
    }

    delay(5);
  }

  gyroBiasX = sumX / samples;
  gyroBiasY = sumY / samples;
  gyroBiasZ = sumZ / samples;

  Serial.println();
  Serial.println("Calibration complete.");
  Serial.println();

  Serial.print("Gyro Bias X : ");
  Serial.print(gyroBiasX, 4);
  Serial.println(" °/s");

  Serial.print("Gyro Bias Y : ");
  Serial.print(gyroBiasY, 4);
  Serial.println(" °/s");

  Serial.print("Gyro Bias Z : ");
  Serial.print(gyroBiasZ, 4);
  Serial.println(" °/s");

  Serial.println("================================");
}

// Applying gyro Bias
void MPU6500::applyGyroCalibration(IMUData &data) {
  data.gx -= gyroBiasX;
  data.gy -= gyroBiasY;
  data.gz -= gyroBiasZ;
}

// Accelerometer Calibration Function
void MPU6500::calibrateAccelerometer(IMUData &data) {
  const int samples = 500;

  float sumX = 0.0f;
  float sumY = 0.0f;
  float sumZ = 0.0f;

  int successfulSamples = 0;

  Serial.println();
  Serial.println("================================");
  Serial.println(" ACCELEROMETER CALIBRATION");
  Serial.println("================================");
  Serial.println("Place the flight controller LEVEL.");
  Serial.println("Keep it completely STILL.");
  Serial.println();

  delay(2000);

  while (successfulSamples < samples) {
    if (readRawData(data)) {
      convertRawData(data);

      sumX += data.ax;
      sumY += data.ay;
      sumZ += data.az;

      successfulSamples++;
    }

    delay(5);
  }

  if (successfulSamples == 0) {
    Serial.println("ERROR: Accelerometer calibration failed.");
    return;
  }

  float averageX = sumX / successfulSamples;
  float averageY = sumY / successfulSamples;
  float averageZ = sumZ / successfulSamples;

  /*
       For a level sensor:

       X expected = 0
       Y expected = 0
       Z expected = +9.80665 m/s²
    */

  accelBiasX = averageX;
  accelBiasY = averageY;
  accelBiasZ = averageZ - GRAVITY;

  Serial.println();
  Serial.println("Accelerometer calibration complete.");
  Serial.println();

  Serial.print("Average AX : ");
  Serial.print(averageX, 4);
  Serial.println(" m/s²");

  Serial.print("Average AY : ");
  Serial.print(averageY, 4);
  Serial.println(" m/s²");

  Serial.print("Average AZ : ");
  Serial.print(averageZ, 4);
  Serial.println(" m/s²");

  Serial.println();

  Serial.print("Accel Bias X : ");
  Serial.print(accelBiasX, 4);
  Serial.println(" m/s²");

  Serial.print("Accel Bias Y : ");
  Serial.print(accelBiasY, 4);
  Serial.println(" m/s²");

  Serial.print("Accel Bias Z : ");
  Serial.print(accelBiasZ, 4);
  Serial.println(" m/s²");

  Serial.println("================================");
}

// Applying accelerometer Bias
void MPU6500::applyAccelerometerCalibration(IMUData &data) {
  data.ax -= accelBiasX;
  data.ay -= accelBiasY;
  data.az -= accelBiasZ;
}

// Implementing the accelerometer calibration
void MPU6500::calibrateAccelerometer6Point(IMUData &data) {
  const int samples = 500;

  float posX = 0.0f;
  float negX = 0.0f;

  float posY = 0.0f;
  float negY = 0.0f;

  float posZ = 0.0f;
  float negZ = 0.0f;

  int successfulSamples;

  //--------------------------------------------------
  // Helper for collecting one orientation
  //--------------------------------------------------

  auto collectSamples = [&](const char *message, char axis, float &average) {
    float sum = 0.0f;
    successfulSamples = 0;

    Serial.println();
    Serial.println("--------------------------------");
    Serial.println(message);
    Serial.println("Keep the board STILL.");
    Serial.println("Starting in 3 seconds...");
    Serial.println("--------------------------------");

    delay(3000);

    for (int i = 0; i < samples; i++) {
      if (readRawData(data)) {
        convertRawData(data);

        float value = 0.0f;

        if (axis == 'X')
          value = data.ax;

        else if (axis == 'Y')
          value = data.ay;

        else if (axis == 'Z')
          value = data.az;

        sum += value;

        successfulSamples++;
      }

      delay(5);
    }

    if (successfulSamples > 0)
      average = sum / successfulSamples;
    else
      average = 0.0f;
  };
  //--------------------------------------------------
  // +X orientation
  //--------------------------------------------------
  collectSamples("POSITION 1: +X UP", 'X', posX);
  //--------------------------------------------------
  // -X orientation
  //--------------------------------------------------
  collectSamples("POSITION 2: -X UP", 'X', negX);
  //--------------------------------------------------
  // +Y orientation
  //--------------------------------------------------
  collectSamples("POSITION 3: +Y UP", 'Y', posY);
  //--------------------------------------------------
  // -Y orientation
  //--------------------------------------------------
  collectSamples("POSITION 4: -Y UP", 'Y', negY);
  //--------------------------------------------------
  // +Z orientation
  //--------------------------------------------------
  collectSamples("POSITION 5: +Z UP", 'Z', posZ);
  //--------------------------------------------------
  // -Z orientation
  //--------------------------------------------------
  collectSamples("POSITION 6: -Z UP", 'Z', negZ);
  //--------------------------------------------------
  // Calculate calibration
  //--------------------------------------------------

  accelBiasX = (posX + negX) / 2.0f;
  accelScaleX = (2.0f * GRAVITY) / (posX - negX);

  accelBiasY = (posY + negY) / 2.0f;
  accelScaleY = (2.0f * GRAVITY) / (posY - negY);

  accelBiasZ = (posZ + negZ) / 2.0f;
  accelScaleZ = (2.0f * GRAVITY) / (posZ - negZ);

  //--------------------------------------------------
  // Print results
  //--------------------------------------------------
  Serial.println();
  Serial.println("================================");
  Serial.println("  6-POINT CALIBRATION COMPLETE");
  Serial.println("================================");

  Serial.println();

  Serial.print("+X average : ");
  Serial.println(posX, 4);

  Serial.print("-X average : ");
  Serial.println(negX, 4);

  Serial.print("+Y average : ");
  Serial.println(posY, 4);

  Serial.print("-Y average : ");
  Serial.println(negY, 4);

  Serial.print("+Z average : ");
  Serial.println(posZ, 4);

  Serial.print("-Z average : ");
  Serial.println(negZ, 4);

  Serial.println();

  Serial.print("Bias X : ");
  Serial.println(accelBiasX, 4);

  Serial.print("Bias Y : ");
  Serial.println(accelBiasY, 4);

  Serial.print("Bias Z : ");
  Serial.println(accelBiasZ, 4);

  Serial.println();

  Serial.print("Scale X : ");
  Serial.println(accelScaleX, 6);

  Serial.print("Scale Y : ");
  Serial.println(accelScaleY, 6);

  Serial.print("Scale Z : ");
  Serial.println(accelScaleZ, 6);

  Serial.println("================================");
}

// Implementing the 6-point calibration function
void MPU6500::calibrateAccelerometer6Point() {
  const int SAMPLES = 500;

  float xPositive = 0.0f;
  float xNegative = 0.0f;

  float yPositive = 0.0f;
  float yNegative = 0.0f;

  float zPositive = 0.0f;
  float zNegative = 0.0f;

  IMUData data;

  Serial.println();
  Serial.println("================================");
  Serial.println("  6-POINT ACCELEROMETER CALIBRATION");
  Serial.println("================================");

  //==================================================
  // +X
  //==================================================
  Serial.println();
  Serial.println("POSITION 1/6 : +X UP");
  Serial.println("Place the flight controller with +X UP.");
  Serial.println("Keep it completely STILL.");

  delay(3000);

  float sum = 0;

  for (int i = 0; i < SAMPLES; i++) {
    if (readRawData(data)) {
      convertRawData(data);
      sum += data.ax;
    }

    delay(2);
  }

  xPositive = sum / SAMPLES;

  Serial.print("Average AX : ");
  Serial.println(xPositive, 4);


  //==================================================
  // -X
  //==================================================
  Serial.println();
  Serial.println("POSITION 2/6 : -X UP");
  Serial.println("Place the flight controller with -X UP.");
  Serial.println("Keep it completely STILL.");

  delay(3000);

  sum = 0;

  for (int i = 0; i < SAMPLES; i++) {
    if (readRawData(data)) {
      convertRawData(data);
      sum += data.ax;
    }

    delay(2);
  }

  xNegative = sum / SAMPLES;

  Serial.print("Average AX : ");
  Serial.println(xNegative, 4);


  //==================================================
  // +Y
  //==================================================
  Serial.println();
  Serial.println("POSITION 3/6 : +Y UP");
  Serial.println("Place the flight controller with +Y UP.");
  Serial.println("Keep it completely STILL.");

  delay(3000);

  sum = 0;

  for (int i = 0; i < SAMPLES; i++) {
    if (readRawData(data)) {
      convertRawData(data);
      sum += data.ay;
    }

    delay(2);
  }

  yPositive = sum / SAMPLES;

  Serial.print("Average AY : ");
  Serial.println(yPositive, 4);


  //==================================================
  // -Y
  //=================================================
  Serial.println();
  Serial.println("POSITION 4/6 : -Y UP");
  Serial.println("Place the flight controller with -Y UP.");
  Serial.println("Keep it completely STILL.");

  delay(3000);

  sum = 0;

  for (int i = 0; i < SAMPLES; i++) {
    if (readRawData(data)) {
      convertRawData(data);
      sum += data.ay;
    }

    delay(2);
  }

  yNegative = sum / SAMPLES;

  Serial.print("Average AY : ");
  Serial.println(yNegative, 4);


  //==================================================
  // +Z
  //==================================================
  Serial.println();
  Serial.println("POSITION 5/6 : +Z UP");
  Serial.println("Place the flight controller with +Z UP.");
  Serial.println("Keep it completely STILL.");

  delay(3000);

  sum = 0;

  for (int i = 0; i < SAMPLES; i++) {
    if (readRawData(data)) {
      convertRawData(data);
      sum += data.az;
    }

    delay(2);
  }

  zPositive = sum / SAMPLES;

  Serial.print("Average AZ : ");
  Serial.println(zPositive, 4);


  //==================================================
  // -Z
  //==================================================
  Serial.println();
  Serial.println("POSITION 6/6 : -Z UP");
  Serial.println("Place the flight controller with -Z UP.");
  Serial.println("Keep it completely STILL.");

  delay(3000);

  sum = 0;

  for (int i = 0; i < SAMPLES; i++) {
    if (readRawData(data)) {
      convertRawData(data);
      sum += data.az;
    }

    delay(2);
  }

  zNegative = sum / SAMPLES;

  Serial.print("Average AZ : ");
  Serial.println(zNegative, 4);


  //==================================================
  // Calculate bias
  //=================================================
  accelBiasX = (xPositive + xNegative) / 2.0f;
  accelBiasY = (yPositive + yNegative) / 2.0f;
  accelBiasZ = (zPositive + zNegative) / 2.0f;
  //==================================================
  // Calculate scale
  //==================================================
  float xSpan = (xPositive - xNegative) / 2.0f;
  float ySpan = (yPositive - yNegative) / 2.0f;
  float zSpan = (zPositive - zNegative) / 2.0f;

  accelScaleX = GRAVITY / xSpan;
  accelScaleY = GRAVITY / ySpan;
  accelScaleZ = GRAVITY / zSpan;


  //==================================================
  // Print results
  //==================================================
  Serial.println();
  Serial.println("================================");
  Serial.println("  6-POINT CALIBRATION COMPLETE");
  Serial.println("================================");

  Serial.print("Accel Bias X : ");
  Serial.println(accelBiasX, 4);

  Serial.print("Accel Bias Y : ");
  Serial.println(accelBiasY, 4);

  Serial.print("Accel Bias Z : ");
  Serial.println(accelBiasZ, 4);

  Serial.println();

  Serial.print("Accel Scale X : ");
  Serial.println(accelScaleX, 6);

  Serial.print("Accel Scale Y : ");
  Serial.println(accelScaleY, 6);

  Serial.print("Accel Scale Z : ");
  Serial.println(accelScaleZ, 6);

  Serial.println("================================");
}

void MPU6500::applyAccelCalibration(IMUData &data) {
  data.ax = (data.ax - accelBiasX) * accelScaleX;
  data.ay = (data.ay - accelBiasY) * accelScaleY;
  data.az = (data.az - accelBiasZ) * accelScaleZ;
}
/*
void MPU6500::printConfiguration() {
  Serial.println();
  Serial.println("========== MPU6500 Configuration ==========");

  Serial.print("PWR_MGMT_1   : 0x");
  Serial.println(readRegister(PWR_MGMT_1), HEX);

  Serial.print("CONFIG       : 0x");
  Serial.println(readRegister(CONFIG), HEX);

  Serial.print("SMPLRT_DIV   : 0x");
  Serial.println(readRegister(SMPLRT_DIV), HEX);

  Serial.print("GYRO_CONFIG  : 0x");
  Serial.println(readRegister(GYRO_CONFIG), HEX);

  Serial.print("ACCEL_CONFIG : 0x");
  Serial.println(readRegister(ACCEL_CONFIG), HEX);

  Serial.print("ACCEL_CFG2   : 0x");
  Serial.println(readRegister(ACCEL_CONFIG2), HEX);

  Serial.println("===========================================");
  Serial.println();
}*/