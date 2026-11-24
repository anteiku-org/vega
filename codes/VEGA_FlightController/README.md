# PROJECT VEGA: High-Performance ESP32 Flight Controller

An autonomous, dual-core, real-time quadcopter flight controller firmware engineered for the **ESP32 DevKit (36-pin)**, inspired by the high-performance architecture of **ESP-FC**.

---

## 1. System Architecture

```
                   ESP32 DUAL-CORE FREERTOS SCHEDULER
                   
       +-------------------------------------------------------+
       | CORE 1: Hard Real-Time Flight Loop (250Hz / 4000us)   |
       |  - Fast I2C Burst Read (MPU6050 Accel + Gyro)         |
       |  - 3D Mahony Quaternion Attitude Fusion (AHRS)        |
       |  - Cascaded PID Controller (Angle -> Rate Loop)       |
       |  - Quad-X Motor Mixer                                 |
       |  - Sub-microsecond 250Hz ESC Pulse Generation         |
       +-------------------------------------------------------+
                                  ^
                                  | Thread-Safe State & Radio Controls
                                  v
       +-------------------------------------------------------+
       | CORE 0: Background Avionics & Communications (50Hz)   |
       |  - LoRa SX1276 433MHz Packet Transceiver              |
       |  - Failsafe Watchdog & Safe-Arming Interlock          |
       |  - Barometer (BMP280) Altitude at 20Hz                |
       |  - Compass (QMC5883L) Heading at 20Hz                 |
       |  - Battery Voltage Monitor (INA219) at 10Hz           |
       |  - GPS (NEO-6M) NMEA UART Stream on Serial2           |
       |  - Ambient Temperature & Humidity (DHT11)             |
       |  - Multi-Pattern Audible Buzzer Alarm Machine         |
       +-------------------------------------------------------+
```

---

## 2. Complete Master Pinout (ESP32 36-Pin)

| Subsystem | Peripheral Module | Module Pin | ESP32 GPIO | Power Rail | Notes |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **I2C Bus** | **MPU6050, BMP280, QMC5883L, INA219** | `SDA` | **GPIO 21** | 3.3V Buck | Fast 400kHz I2C Bus |
| | | `SCL` | **GPIO 22** | 3.3V Buck | Shared Clock |
| **LoRa Radio**| **SX1276 / Ra-02** | `SCK` | **GPIO 18** | 3.3V Buck | VSPI Clock |
| | | `MISO`| **GPIO 19** | 3.3V Buck | VSPI MISO |
| | | `MOSI`| **GPIO 23** | 3.3V Buck | VSPI MOSI |
| | | `NSS` | **GPIO 5** | 3.3V Buck | Chip Select |
| | | `RST` | **GPIO 14** | 3.3V Buck | Reset |
| | | `DIO0`| **GPIO 26** | 3.3V Buck | Packet Interrupt |
| **GPS** | **NEO-6M GPS** | `TX` | **GPIO 16** | 5V Buck | ESP32 RX2 |
| | | `RX` | **GPIO 17** | 5V Buck | ESP32 TX2 |
| **Motors** | **ESC 1 (Rear-Right, CCW)** | `SIG` | **GPIO 32** | Main LiPo | 250Hz Sub-µs PWM |
| | **ESC 2 (Front-Right, CW)** | `SIG` | **GPIO 33** | Main LiPo | 250Hz Sub-µs PWM |
| | **ESC 3 (Rear-Left, CW)** | `SIG` | **GPIO 25** | Main LiPo | 250Hz Sub-µs PWM |
| | **ESC 4 (Front-Left, CCW)** | `SIG` | **GPIO 27** | Main LiPo | 250Hz Sub-µs PWM |
| **Auxiliary**| **DHT11** | `DATA`| **GPIO 4** | 3.3V Buck | 1-Wire Ambient Data |
| | **Buzzer** | `I/O` | **GPIO 13** | 5V / 3.3V | Audible Alarm |

---

## 3. How to Compile & Flash

### Option A: PlatformIO (Recommended)
1. Open this folder in VS Code with PlatformIO installed.
2. Connect your ESP32 via USB.
3. Click **PlatformIO: Build** or run `pio run -t upload`.

### Option B: Arduino IDE
1. Open `VEGA_FlightController.ino` in Arduino IDE.
2. Select Board: **ESP32 Dev Module**.
3. Set CPU Frequency: **240MHz (WiFi/BT)**.
4. Set Upload Speed: **921600** or **115200**.
5. Click **Upload**.

---

## 4. Safety & Arming Procedure

1. **Pre-Arming Safety Check**:
   - The drone powers up in **DISARMED** mode. Motors will not spin.
   - You cannot arm the drone if the throttle stick is above `1050µs`.
2. **Arming Confirmation**:
   - When armed, the buzzer plays a solid **250ms confirmation tone**, and motors enter idle spool-up (`1050µs`).
3. **Failsafe System**:
   - If LoRa radio communication drops for more than **500ms**, the failsafe immediately disarms the motors, forces throttle to zero, and triggers the continuous alarm buzzer.
