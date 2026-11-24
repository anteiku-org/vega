#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// ESP32 I2C Pins
#define I2C_SDA 21
#define I2C_SCL 22

// Create an instance of the BMP280 sensor
Adafruit_BMP280 bmp; // Uses I2C

void setup() {
  Serial.begin(115200);
  Serial.println(F("BMP280 Individual Test"));
  
  // Initialize I2C with specific pins for ESP32
  Wire.begin(I2C_SDA, I2C_SCL);

  // Initialize the sensor.
  // Note: Address 0x76 is common for these modules. Sometimes it is 0x77.
  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor at address 0x76!"));
    Serial.println(F("Trying address 0x77 instead..."));
    
    if (!bmp.begin(0x77)) {
      Serial.println(F("Could not find a valid BMP280 sensor at 0x77 either."));
      Serial.println(F("Please check your wiring (VCC to 3.3V, GND, SDA to 21, SCL to 22)."));
      Serial.println(F("If your module has SDO, connect it to GND. If it has CSB, connect it to 3.3V."));
      while (1) delay(10); // Halt execution
    }
  }

  Serial.println("BMP280 connected successfully!");

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void loop() {
  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure() / 100.0F); // Convert Pa to hPa
  Serial.println(" hPa");

  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! 1013.25 is standard sea level pressure */
  Serial.println(" m");

  Serial.println("-----------------------------------------");
  delay(1000); // Wait 1 second before next reading
}
