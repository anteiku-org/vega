# Individual Sensor Testing Guide

This folder contains isolated testing sketches for your sensors to verify they are functioning correctly before integrating them into the larger flight controller code. 

Always test one sensor at a time to rule out bus conflicts.

## Pin Mapping (ESP32 DevKit V1)

Both sensors use the **I2C protocol** and can technically be connected to the same pins. However, to test them individually, only connect **one** at a time to the ESP32.

| Sensor Pin | ESP32 Pin | Purpose |
| :--- | :--- | :--- |
| **VCC** / **VIN** | **3.3V** | Power (Do NOT use 5V) |
| **GND** | **GND** | Ground |
| **SDA** | **GPIO 21** | I2C Data Line |
| **SCL** | **GPIO 22** | I2C Clock Line |

### Special notes for BMP280
If your BMP280 module has more than 4 pins, you MUST wire these correctly for I2C to work:
- **CSB** (Chip Select): Connect to **3.3V** (This forces the chip into I2C mode instead of SPI).
- **SDO** (Serial Data Out): Connect to **GND** (This sets the I2C address firmly to `0x76`).

---

## 1. Testing the MPU6050
1. Disconnect the BMP280. Connect **ONLY** the MPU6050 to the ESP32.
2. Navigate to the `MPU6050_test` folder and open `MPU6050_test.ino` in the Arduino IDE.
3. Upload the code.
4. Open the Serial Monitor and set the baud rate to `115200`.
5. You should see raw Accelerometer, Gyroscope, and Temperature data updating every half second. If you move the sensor, the values should change rapidly.

## 2. Testing the BMP280
The BMP280 code uses the standard Adafruit library to simplify the complex math required for this sensor.
1. In the Arduino IDE, go to **Sketch > Include Library > Manage Libraries...**
2. Search for and install **Adafruit BMP280 Library** (If it asks to install dependencies like *Adafruit Unified Sensor*, click **Install All**).
3. Disconnect the MPU6050. Connect **ONLY** the BMP280 to the ESP32.
4. Navigate to the `BMP280_test` folder and open `BMP280_test.ino` in the Arduino IDE.
5. Upload the code.
6. Open the Serial Monitor and set the baud rate to `115200`.
7. You should see Temperature, Pressure, and approximate Altitude readings updating every second.

---
### Troubleshooting
- **"Could not connect..."**: Check your jumper wires. Try using different wires, as breadboard wires often break inside the plastic.
- **Nothing prints in Serial Monitor**: Ensure the baud rate in the bottom right corner of the Serial Monitor is set to `115200`.
