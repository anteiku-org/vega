#include <Wire.h>

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050

int16_t accel_x, accel_y, accel_z; // Raw Accelerometer data
int16_t gyro_x, gyro_y, gyro_z;    // Raw Gyroscope data
int16_t tempRaw;                   // Raw Temperature

// Variables to hold converted data
float ax, ay, az;
float gx, gy, gz;

void setup() {
  Serial.begin(115200);
  Wire.begin(); // SDA=21, SCL=22 on ESP32
  
  Serial.println(F("Initializing MPU6050..."));
  
  // 1. WAKE UP THE MPU6050
  // By default, it starts in sleep mode to save power.
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Power Management 1 register
  Wire.write(0x00); // Write 0 to wake it up
  Wire.endTransmission(true);

  // 2. CONFIGURE THE GYROSCOPE FULL-SCALE RANGE
  // Register 0x1B: Gyroscope Configuration
  // 0x00 = ±250°/s, 0x08 = ±500°/s, 0x10 = ±1000°/s, 0x18 = ±2000°/s
  // For drones, doing a flip takes about 1 second (360°/s). So ±500°/s is a standard safe limit.
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); 
  Wire.write(0x08); 
  Wire.endTransmission(true);

  // 3. CONFIGURE THE ACCELEROMETER FULL-SCALE RANGE
  // Register 0x1C: Accelerometer Configuration
  // 0x00 = ±2g, 0x08 = ±4g, 0x10 = ±8g, 0x18 = ±16g
  // Drone motors cause heavy high-g vibrations. ±2g will "clip" easily. We use ±8g.
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); 
  Wire.write(0x10); 
  Wire.endTransmission(true);

  // 4. HARDWARE DIGITAL LOW PASS FILTER (DLPF)
  // Register 0x1A: Configuration. 
  // THIS IS CRITICAL FOR DRONES! It filters out high-frequency motor vibrations before they even reach the ESP32.
  // 0x00 = Filter off, 0x03 = ~42Hz bandwidth filter. Smooths out motor noise perfectly.
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A); 
  Wire.write(0x03); 
  Wire.endTransmission(true);

  Serial.println(F("MPU6050 Ready! Make sure the sensor is perfectly flat and still."));
  delay(1000);
}

void loop() {
  // Read 14 bytes starting from register 0x3B (ACCEL_XOUT_H)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); 

  // Reading sequence MUST match the hardware register map: Accel (6 bytes), Temp (2 bytes), Gyro (6 bytes)
  accel_x = Wire.read()<<8 | Wire.read(); 
  accel_y = Wire.read()<<8 | Wire.read(); 
  accel_z = Wire.read()<<8 | Wire.read(); 
  
  tempRaw = Wire.read()<<8 | Wire.read(); // We can ignore temperature for now
  
  gyro_x = Wire.read()<<8 | Wire.read(); 
  gyro_y = Wire.read()<<8 | Wire.read(); 
  gyro_z = Wire.read()<<8 | Wire.read(); 

  // --- CONVERTING RAW DATA TO REAL PHYSICS UNITS ---
  
  // For ±8g setting, the datasheet tells us the scale factor is 4096 LSB/g
  ax = (float)accel_x / 4096.0;
  ay = (float)accel_y / 4096.0;
  az = (float)accel_z / 4096.0;

  // For ±500°/s setting, the datasheet tells us the scale factor is 65.5 LSB/(°/s)
  gx = (float)gyro_x / 65.5;
  gy = (float)gyro_y / 65.5;
  gz = (float)gyro_z / 65.5;

  // Print Accel data (in 'g's of force)
  Serial.print("Acc [X: "); Serial.print(ax, 2);
  Serial.print(" Y: "); Serial.print(ay, 2);
  Serial.print(" Z: "); Serial.print(az, 2);
  Serial.print("] g  |  ");
  
  // Print Gyro data (in degrees per second)
  Serial.print("Gyro [X: "); Serial.print(gx, 1);
  Serial.print(" Y: "); Serial.print(gy, 1);
  Serial.print(" Z: "); Serial.print(gz, 1);
  Serial.println("] deg/s");
  
  // A real flight controller loop runs at 4ms (250Hz), but we slow it down so we can read the serial monitor
  delay(100); 
}
