#include <Wire.h>
#include <math.h>

// I2C Addresses for the two different chips used on GY-271 modules
#define HMC5883L_ADDR 0x1E
#define QMC5883L_ADDR 0x0D

uint8_t mag_addr = 0x00;
bool isQMC = false;

void setup() {
  Serial.begin(115200);
  Wire.begin(); // SDA = 21, SCL = 22 on ESP32

  Serial.println(F("Initializing GY-271 Magnetometer..."));
  
  // 1. THE GREAT CLONE DETECTOR
  // We scan the I2C bus to figure out which chip you actually have!
  Wire.beginTransmission(QMC5883L_ADDR);
  if (Wire.endTransmission() == 0) {
    mag_addr = QMC5883L_ADDR;
    isQMC = true;
    Serial.println(F("Detected: QMC5883L (Newer Clone) at 0x0D"));
  } else {
    Wire.beginTransmission(HMC5883L_ADDR);
    if (Wire.endTransmission() == 0) {
      mag_addr = HMC5883L_ADDR;
      isQMC = false;
      Serial.println(F("Detected: HMC5883L (Original Honeywell) at 0x1E"));
    }
  }

  if (mag_addr == 0x00) {
    Serial.println(F("ERROR: No magnetometer found! Check wiring. (3.3V, GND, SCL->22, SDA->21)"));
    while(1) delay(10); // Halt
  }

  // 2. REGISTER-LEVEL CONFIGURATION
  // We don't use libraries here! We talk directly to the chip's brain via registers.
  if (isQMC) {
    // QMC5883L Configuration
    Wire.beginTransmission(mag_addr);
    Wire.write(0x09); // Control Register 1
    // 0x1D = 0b00011101 : 
    // OSR = 512 (less noise), RNG = 2G (high precision), ODR = 200Hz (fast), MODE = Continuous
    Wire.write(0x1D); 
    Wire.endTransmission();
    
    Wire.beginTransmission(mag_addr);
    Wire.write(0x0B); // Set/Reset Period Register
    Wire.write(0x01); // Recommended by QMC datasheet
    Wire.endTransmission();
  } else {
    // HMC5883L Configuration
    Wire.beginTransmission(mag_addr);
    Wire.write(0x00); // Config Register A
    Wire.write(0x70); // 8 samples averaged, 15Hz output rate
    Wire.endTransmission();
    
    Wire.beginTransmission(mag_addr);
    Wire.write(0x01); // Config Register B
    Wire.write(0xA0); // Gain setting (resolution)
    Wire.endTransmission();

    Wire.beginTransmission(mag_addr);
    Wire.write(0x02); // Mode Register
    Wire.write(0x00); // Continuous measurement mode
    Wire.endTransmission();
  }
}

void loop() {
  int16_t x, y, z;

  // 3. READING RAW MAGNETIC VECTORS
  if (isQMC) {
    Wire.beginTransmission(mag_addr);
    Wire.write(0x00); // Start reading at X_LSB register
    Wire.endTransmission();
    Wire.requestFrom((int)mag_addr, 6); // Request 6 bytes (X, Y, Z are 2 bytes each)
    if (Wire.available() == 6) {
      // QMC sends data as LSB first, then MSB
      x = Wire.read() | (Wire.read() << 8);
      y = Wire.read() | (Wire.read() << 8);
      z = Wire.read() | (Wire.read() << 8);
    }
  } else {
    Wire.beginTransmission(mag_addr);
    Wire.write(0x03); // Start reading at Data Output X MSB register
    Wire.endTransmission();
    Wire.requestFrom((int)mag_addr, 6);
    if (Wire.available() == 6) {
      // HMC sends data as MSB first, then LSB. And order is X, Z, Y!
      x = (Wire.read() << 8) | Wire.read();
      z = (Wire.read() << 8) | Wire.read(); 
      y = (Wire.read() << 8) | Wire.read();
    }
  }

  // 4. CALCULATING HEADING (Azimuth)
  // atan2 calculates the angle based on the X and Y vectors.
  float heading = atan2(y, x);
  
  // NOTE: This assumes the sensor is perfectly flat. 
  // If the drone tilts, this calculation becomes inaccurate. 
  // That's why we eventually combine this with the MPU6050 (Tilt-Compensated Compass)

  // Correct for Magnetic Declination (The difference between True North and Magnetic North)
  // You need to find the declination for your specific city on the internet.
  // For example, in New Delhi, India it is roughly +1 degree (0.017 radians).
  float declinationAngle = 0.017; 
  heading += declinationAngle;

  // Ensure heading stays between 0 and 2*PI (0 to 360 degrees)
  if(heading < 0) heading += 2 * PI;
  if(heading > 2 * PI) heading -= 2 * PI;

  // Convert radians to degrees
  float headingDegrees = heading * 180 / PI;

  Serial.print("Raw X: "); Serial.print(x);
  Serial.print("\tY: "); Serial.print(y);
  Serial.print("\tZ: "); Serial.print(z);
  Serial.print("\t| Compass Heading: "); Serial.print(headingDegrees);
  Serial.println("°");

  delay(100);
}
