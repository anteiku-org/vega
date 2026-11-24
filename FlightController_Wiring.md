# Flight Controller Wiring Guide

Here is the explicit pin mapping for your **ESP32 DevKit V1** connected to the **MPU6050** and **BMP280** sensors based on the configuration we just set up.

## I2C Sensor Bus (Shared)
Both the MPU6050 and BMP280 communicate using the I2C protocol. Because I2C is a "bus", you can safely connect multiple sensors to the exact same pins on the ESP32. They won't interfere with each other because they have different hardware addresses (MPU6050 is `0x68`, BMP280 is `0x76`).

| Sensor Pin | ESP32 DevKit V1 Pin | Notes |
| :--- | :--- | :--- |
| **VCC** / **VIN** | **3.3V** | Do **not** use the 5V (VIN) pin. These sensors operate at 3.3V. |
| **GND** | **GND** | Connect to any available Ground pin on the ESP32. |
| **SDA** (Data) | **GPIO 21** | Connect the SDA pins of BOTH sensors to GPIO 21. |
| **SCL** (Clock) | **GPIO 22** | Connect the SCL pins of BOTH sensors to GPIO 22. |

> [!CAUTION]
> **Power Supply Warning**
> Double-check that your sensors are connected to the `3.3V` output of the ESP32 and **not** the `5V` (or `VIN`) pin. Applying 5V to the I2C pins of the MPU6050 or BMP280 can permanently damage them.

## Future Expansion
When you are ready to add the remaining components (like the LoRa radio, GPS, and ESCs), you can reference the complete master blueprint we created earlier in the `ESP32_PINOUT_AND_SAFETY_GUIDE.md` file located in your project folder.
