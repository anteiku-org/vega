#include <Wire.h>

// ESP32 I2C Pins
#define I2C_SDA 21
#define I2C_SCL 23

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C with specific pins for ESP32
  Wire.begin(I2C_SDA, I2C_SCL);
  
  Serial.println("Initializing I2C and waking up MPU6050...");
  
  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  byte status = Wire.endTransmission(true);
  
  if (status == 0) {
    Serial.println("MPU6050 connected successfully!");
  } else {
    Serial.println("Could not connect to MPU6050.");
    Serial.print("I2C Error Code: ");
    Serial.println(status);
    Serial.println("Please check your wiring (VCC to 3.3V, GND, SDA to 21, SCL to 22).");
    while(1); // Halt execution
  }
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  
  // Request 14 registers (2 for each axis of accel, temp, and gyro)
  Wire.requestFrom(MPU_ADDR, 14, true);  
  
  if (Wire.available() == 14) {
    // Read registers sequentially
    AcX = Wire.read()<<8 | Wire.read();  
    AcY = Wire.read()<<8 | Wire.read();  
    AcZ = Wire.read()<<8 | Wire.read();  
    Tmp = Wire.read()<<8 | Wire.read();  
    GyX = Wire.read()<<8 | Wire.read();  
    GyY = Wire.read()<<8 | Wire.read();  
    GyZ = Wire.read()<<8 | Wire.read();  
    
    // Print out raw values
    Serial.print("Accelerometer: ");
    Serial.print("X = "); Serial.print(AcX);
    Serial.print(" | Y = "); Serial.print(AcY);
    Serial.print(" | Z = "); Serial.println(AcZ);
    
    Serial.print("Gyroscope: ");
    Serial.print("X = "); Serial.print(GyX);
    Serial.print(" | Y = "); Serial.print(GyY);
    Serial.print(" | Z = "); Serial.println(GyZ);
    
    // Convert raw temperature to Celsius
    Serial.print("Temperature: "); 
    Serial.print(Tmp / 340.00 + 36.53); Serial.println(" C");
    Serial.println("-----------------------------------------");
  }
  
  delay(500); // Wait half a second before next reading
}
