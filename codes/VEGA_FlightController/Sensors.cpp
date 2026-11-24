#include "Sensors.h"

SensorSuite::SensorSuite()
    : gyroBiasX(0.0f), gyroBiasY(0.0f), gyroBiasZ(0.0f),
      baselinePressure(1013.25f), nmeaBuffer("") {
    data.mpuReady = false;
    data.bmpReady = false;
    data.qmcReady = false;
    data.inaReady = false;
    data.gpsFix = false;
    data.gpsSatellites = 0;
    data.gpsLatitude = 0.0f;
    data.gpsLongitude = 0.0f;
    data.gpsAltitude = 0.0f;
    data.gpsSpeedKmh = 0.0f;
    data.batteryVoltage = 12.0f;
    data.pressureHpa = 1013.25f;
    data.altitudeM = 0.0f;
    data.temperatureC = 25.0f;
    data.compassHeading = 0.0f;
    data.dhtTemperature = 25.0f;
    data.dhtHumidity = 50.0f;
    data.dhtReady = false;
}

void SensorSuite::writeRegister8(uint8_t devAddr, uint8_t regAddr, uint8_t value) {
    Wire.beginTransmission(devAddr);
    Wire.write(regAddr);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t SensorSuite::readRegister8(uint8_t devAddr, uint8_t regAddr) {
    Wire.beginTransmission(devAddr);
    Wire.write(regAddr);
    Wire.endTransmission(false);
    Wire.requestFrom((int)devAddr, 1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0;
}

bool SensorSuite::beginI2C() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQUENCY);
    delay(50);

    // 1. Initialize MPU6050
    Wire.beginTransmission(ADDR_MPU6050);
    if (Wire.endTransmission() == 0) {
        writeRegister8(ADDR_MPU6050, 0x6B, 0x00); // Wake up device
        delay(10);
        writeRegister8(ADDR_MPU6050, 0x1A, 0x03); // DLPF: 42Hz filter
        writeRegister8(ADDR_MPU6050, 0x1B, 0x18); // Gyro: +/- 2000 deg/s (16.4 LSB/deg/s)
        writeRegister8(ADDR_MPU6050, 0x1C, 0x10); // Accel: +/- 8g (4096 LSB/g)
        data.mpuReady = true;
    }

    // 2. Initialize BMP280
    Wire.beginTransmission(ADDR_BMP280);
    if (Wire.endTransmission() == 0) {
        data.bmpReady = true;
        // Read 24 calibration bytes starting at 0x88
        Wire.beginTransmission(ADDR_BMP280);
        Wire.write(0x88);
        Wire.endTransmission(false);
        Wire.requestFrom((int)ADDR_BMP280, 24);
        if (Wire.available() >= 24) {
            dig_T1 = Wire.read() | (Wire.read() << 8);
            dig_T2 = Wire.read() | (Wire.read() << 8);
            dig_T3 = Wire.read() | (Wire.read() << 8);
            dig_P1 = Wire.read() | (Wire.read() << 8);
            dig_P2 = Wire.read() | (Wire.read() << 8);
            dig_P3 = Wire.read() | (Wire.read() << 8);
            dig_P4 = Wire.read() | (Wire.read() << 8);
            dig_P5 = Wire.read() | (Wire.read() << 8);
            dig_P6 = Wire.read() | (Wire.read() << 8);
            dig_P7 = Wire.read() | (Wire.read() << 8);
            dig_P8 = Wire.read() | (Wire.read() << 8);
            dig_P9 = Wire.read() | (Wire.read() << 8);
        }
        writeRegister8(ADDR_BMP280, 0xF4, 0x27); // Normal mode, temp x1, press x1
        writeRegister8(ADDR_BMP280, 0xF5, 0xA0); // Standby 1000ms, filter 4
    }

    // 3. Initialize QMC5883L
    Wire.beginTransmission(ADDR_QMC5883L);
    if (Wire.endTransmission() == 0) {
        writeRegister8(ADDR_QMC5883L, 0x0B, 0x01); // Set/Reset Period
        writeRegister8(ADDR_QMC5883L, 0x09, 0x1D); // Continuous mode, 100Hz, 8G range, 512 OSR
        data.qmcReady = true;
    }

    // 4. Initialize INA219
    Wire.beginTransmission(ADDR_INA219);
    if (Wire.endTransmission() == 0) {
        data.inaReady = true;
    }

    // 5. Initialize Hardware Serial 2 for GPS
    Serial2.begin(GPS_BAUD_RATE, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

    return data.mpuReady;
}

bool SensorSuite::calibrateGyro(int samples) {
    if (!data.mpuReady) return false;

    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    for (int i = 0; i < samples; ++i) {
        readFastIMU();
        sumX += data.gyroRate.x;
        sumY += data.gyroRate.y;
        sumZ += data.gyroRate.z;
        delay(4);
    }
    gyroBiasX = sumX / (float)samples;
    gyroBiasY = sumY / (float)samples;
    gyroBiasZ = sumZ / (float)samples;
    return true;
}

void SensorSuite::readFastIMU() {
    if (!data.mpuReady) return;

    Wire.beginTransmission(ADDR_MPU6050);
    Wire.write(0x3B); // Starting register: ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom((int)ADDR_MPU6050, 14, true);

    if (Wire.available() >= 14) {
        int16_t ax = (Wire.read() << 8) | Wire.read();
        int16_t ay = (Wire.read() << 8) | Wire.read();
        int16_t az = (Wire.read() << 8) | Wire.read();
        Wire.read(); Wire.read(); // Skip temperature
        int16_t gx = (Wire.read() << 8) | Wire.read();
        int16_t gy = (Wire.read() << 8) | Wire.read();
        int16_t gz = (Wire.read() << 8) | Wire.read();

        // Scale factors: Accel +/-8g = 4096 LSB/g; Gyro +/-2000 deg/s = 16.4 LSB/deg/s
        data.accelG.x = (float)ax / 4096.0f;
        data.accelG.y = (float)ay / 4096.0f;
        data.accelG.z = (float)az / 4096.0f;

        float rawGx = (float)gx / 16.4f;
        float rawGy = (float)gy / 16.4f;
        float rawGz = (float)gz / 16.4f;

        // Subtract calibrated biases
        data.gyroRate.x = rawGx - gyroBiasX;
        data.gyroRate.y = rawGy - gyroBiasY;
        data.gyroRate.z = rawGz - gyroBiasZ;
    }
}

void SensorSuite::readBarometer() {
    if (!data.bmpReady) return;

    Wire.beginTransmission(ADDR_BMP280);
    Wire.write(0xF7); // Press & Temp registers
    Wire.endTransmission(false);
    Wire.requestFrom((int)ADDR_BMP280, 6);

    if (Wire.available() >= 6) {
        int32_t adc_P = ((uint32_t)Wire.read() << 12) | ((uint32_t)Wire.read() << 4) | ((uint32_t)Wire.read() >> 4);
        int32_t adc_T = ((uint32_t)Wire.read() << 12) | ((uint32_t)Wire.read() << 4) | ((uint32_t)Wire.read() >> 4);

        int32_t var1_T = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
        int32_t var2_T = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
        int32_t t_fine = var1_T + var2_T;
        data.temperatureC = (float)((t_fine * 5 + 128) >> 8) / 100.0f;

        int64_t var1_P = ((int64_t)t_fine) - 128000;
        int64_t var2_P = var1_P * var1_P * (int64_t)dig_P6;
        var2_P = var2_P + ((var1_P * (int64_t)dig_P5) << 17);
        var2_P = var2_P + (((int64_t)dig_P4) << 35);
        var1_P = ((var1_P * var1_P * (int64_t)dig_P3) >> 8) + ((var1_P * (int64_t)dig_P2) << 12);
        var1_P = (((((int64_t)1) << 47) + var1_P)) * ((int64_t)dig_P1) >> 33;

        if (var1_P != 0) {
            int64_t p = 1048576 - adc_P;
            p = (((p << 31) - var2_P) * 3125) / var1_P;
            var1_P = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
            var2_P = (((int64_t)dig_P8) * p) >> 19;
            p = ((p + var1_P + var2_P) >> 8) + (((int64_t)dig_P7) << 4);
            data.pressureHpa = (float)p / 256.0f / 100.0f;
            data.altitudeM = 44330.0f * (1.0f - powf(data.pressureHpa / baselinePressure, 0.1903f));
        }
    }
}

void SensorSuite::readCompass() {
    if (!data.qmcReady) return;

    Wire.beginTransmission(ADDR_QMC5883L);
    Wire.write(0x00);
    Wire.endTransmission(false);
    Wire.requestFrom((int)ADDR_QMC5883L, 6);

    if (Wire.available() >= 6) {
        int16_t mx = Wire.read() | (Wire.read() << 8);
        int16_t my = Wire.read() | (Wire.read() << 8);
        int16_t mz = Wire.read() | (Wire.read() << 8);
        (void)mz;

        float heading = atan2f((float)my, (float)mx) * 180.0f / 3.14159265f;
        if (heading < 0.0f) heading += 360.0f;
        data.compassHeading = heading;
    }
}

void SensorSuite::readBattery() {
    if (!data.inaReady) return;

    Wire.beginTransmission(ADDR_INA219);
    Wire.write(0x02); // Bus Voltage Register
    Wire.endTransmission(false);
    Wire.requestFrom((int)ADDR_INA219, 2);

    if (Wire.available() >= 2) {
        uint16_t raw = (Wire.read() << 8) | Wire.read();
        data.batteryVoltage = (float)(raw >> 3) * 0.004f; // 4mV per LSB
    }
}

void SensorSuite::updateGPS() {
    while (Serial2.available()) {
        char c = (char)Serial2.read();
        if (c == '\n') {
            parseNMEA(nmeaBuffer);
            nmeaBuffer = "";
        } else if (c != '\r') {
            if (nmeaBuffer.length() < 120) {
                nmeaBuffer += c;
            }
        }
    }
}

void SensorSuite::parseNMEA(const String& s) {
    if (s.startsWith("$GPGGA") || s.startsWith("$GNGGA")) {
        // Parse satellite count and fix
        int comma = 0;
        int lastPos = 0;
        String fields[15];
        for (int i = 0; i < s.length() && comma < 14; i++) {
            if (s[i] == ',') {
                fields[comma++] = s.substring(lastPos, i);
                lastPos = i + 1;
            }
        }
        if (comma >= 9) {
            int fix = fields[6].toInt();
            data.gpsFix = (fix > 0);
            data.gpsSatellites = fields[7].toInt();
            data.gpsAltitude = fields[9].toFloat();
        }
    }
}

void SensorSuite::readDHT11() {
    // Non-blocking simple DHT11 pulse sample on PIN_DHT11
    uint8_t bits[5] = {0, 0, 0, 0, 0};
    pinMode(PIN_DHT11, OUTPUT);
    digitalWrite(PIN_DHT11, LOW);
    delay(20);
    digitalWrite(PIN_DHT11, HIGH);
    delayMicroseconds(30);
    pinMode(PIN_DHT11, INPUT_PULLUP);

    unsigned long t0 = micros();
    while (digitalRead(PIN_DHT11) == HIGH) {
        if (micros() - t0 > 100) return;
    }
    t0 = micros();
    while (digitalRead(PIN_DHT11) == LOW) {
        if (micros() - t0 > 100) return;
    }
    t0 = micros();
    while (digitalRead(PIN_DHT11) == HIGH) {
        if (micros() - t0 > 100) return;
    }

    for (int i = 0; i < 40; ++i) {
        t0 = micros();
        while (digitalRead(PIN_DHT11) == LOW) {
            if (micros() - t0 > 100) return;
        }
        unsigned long tStart = micros();
        while (digitalRead(PIN_DHT11) == HIGH) {
            if (micros() - tStart > 150) return;
        }
        if ((micros() - tStart) > 40) {
            bits[i / 8] |= (1 << (7 - (i % 8)));
        }
    }

    if ((bits[0] + bits[1] + bits[2] + bits[3]) == bits[4]) {
        data.dhtHumidity = (float)bits[0];
        data.dhtTemperature = (float)bits[2];
        data.dhtReady = true;
    }
}
