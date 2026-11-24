#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// Create an instance of the BMP280 object for I2C communication
Adafruit_BMP280 bmp;

// Variables for calibration and relative altitude
float groundLevelPressure = 0.0;
float currentRelativeAltitude = 0.0;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(100);   // wait for serial monitor to open

  Serial.println(F("BMP280 Initialization..."));

  // Initialize the sensor
  unsigned status = bmp.begin(0x76); 
  
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(), 16);
    while (1) delay(10); // Halt the program here if sensor is not found
  }

  // --- BMP280 CONFIGURATION SETTINGS ---
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* IIR Filtering */
                  Adafruit_BMP280::STANDBY_MS_1);   /* Standby time */
                  
  Serial.println(F("BMP280 initialized successfully!"));

  // --- CALIBRATION PHASE (Zeroing the Altitude) ---
  // In a drone, the absolute altitude above sea level doesn't matter for hovering.
  // We care about height above the ground.
  Serial.println(F("Calibrating Ground Level Pressure. Please do not move the drone..."));
  
  float pressureSum = 0;
  int numSamples = 50; // Read 50 times to get a stable average
  
  // We throw away the first few readings as the sensor stabilizes 
  bmp.readPressure();
  delay(100);

  // Take multiple samples to average out any sensor noise
  for(int i = 0; i < numSamples; i++) {
    pressureSum += bmp.readPressure();
    delay(20); // 20ms between readings
  }
  
  // Average the readings
  groundLevelPressure = pressureSum / numSamples;
  
  // Convert from Pascals (Pa) to hectoPascals (hPa) because readAltitude expects hPa
  groundLevelPressure = groundLevelPressure / 100.0; 
  
  Serial.print(F("Calibration Complete! Ground level pressure set to: "));
  Serial.print(groundLevelPressure);
  Serial.println(F(" hPa"));
  Serial.println(F("Starting measurements..."));
  delay(1000);
}

void loop() {
    // Read Temperature and Pressure just to show off!
    Serial.print(F("Temp: "));
    Serial.print(bmp.readTemperature());
    Serial.print(F(" *C  |  "));

    Serial.print(F("Pressure: "));
    Serial.print(bmp.readPressure());
    Serial.print(F(" Pa  |  "));

    // We pass our dynamically calculated groundLevelPressure instead of the standard 1013.25.
    // This gives us our RELATIVE altitude (height from the ground) starting at 0.00 meters!
    currentRelativeAltitude = bmp.readAltitude(groundLevelPressure);

    Serial.print(F("Relative Alt: "));
    Serial.print(currentRelativeAltitude); 
    Serial.println(" m");

    // Small delay just for serial monitor readability.
    // In actual flight controller loop, you run non-blocking code without delay().
    delay(200); 
}
