# PROJECT VEGA: ESP32 Flight Controller & Receiver Architecture Guide

**Project:** Autonomous/Telemetry Quadcopter Drone with LoRa Communication  
**Transmitter (Flight Controller):** ESP32 DevKit V1 (30-Pin NodeMCU Form Factor)  
**Receiver (Ground Station):** Seeed Studio XIAO ESP32S3 Sense  
**Document Type:** Hardware Architecture, Pinout Plan & Avionics Safety Manual  

---

## 1. Executive Summary & Pin Feasibility

Yes, all components listed in the project budget can comfortably operate simultaneously on a **30-pin ESP32 DevKit**. 

By leveraging the **shared I2C bus** for 3 sensors, hardware **VSPI** for LoRa, dedicated **UART2** for GPS, and **LEDC / MCPWM** outputs for the 4 ESCs, the flight controller consumes **14 GPIO pins**, leaving spare general-purpose pins and analog input pins for critical enhancements like LiPo battery voltage sensing and a lost-model buzzer.

---

## 2. Complete Master Pinout Tables

### 2.1 Flight Controller Pinout (ESP32 DevKit V1)

| Subsystem / Bus | ESP32 GPIO | Target Module Pin | Protocol / Signal Type | Description & Purpose |
| :--- | :--- | :--- | :--- | :--- |
| **I2C Bus** *(Shared)* | **GPIO 21** | MPU-6050 SDA<br>QMC5883L SDA<br>BMP388 SDA | I2C Data (Bidirectional) | Shared telemetry data line |
| | **GPIO 22** | MPU-6050 SCL<br>QMC5883L SCL<br>BMP388 SCL | I2C Clock | Shared bus clock line (400 kHz Fast-Mode) |
| **SPI Bus (LoRa)** | **GPIO 18** | SX1278 (Ra-02) SCK | Hardware SPI Clock | Default VSPI Clock |
| | **GPIO 19** | SX1278 (Ra-02) MISO | Hardware SPI MISO | Master-In Slave-Out |
| | **GPIO 23** | SX1278 (Ra-02) MOSI | Hardware SPI MOSI | Master-Out Slave-In |
| | **GPIO 4** | SX1278 (Ra-02) NSS / CS | Standard Output (GPIO) | Chip Select / Slave Select |
| | **GPIO 14** | SX1278 (Ra-02) RST | Standard Output (GPIO) | Radio Hardware Reset line |
| | **GPIO 26** | SX1278 (Ra-02) DIO0 | External Interrupt (GPIO) | RxDone / TxDone Interrupt (Matches test code) |
| **UART Bus (GPS)** | **GPIO 16** | NEO-6M TX | Hardware Serial2 RX | ESP32 listens to NMEA sentences |
| | **GPIO 17** | NEO-6M RX | Hardware Serial2 TX | ESP32 sends UBX configuration commands |
| **Propulsion (ESCs)**| **GPIO 25** | Front-Right ESC Signal | PWM Output (LEDC / MCPWM)| Motor 1 Throttle Control |
| | **GPIO 27** | Rear-Left ESC Signal | PWM Output (LEDC / MCPWM)| Motor 2 Throttle Control |
| | **GPIO 32** | Front-Left ESC Signal | PWM Output (LEDC / MCPWM)| Motor 3 Throttle Control |
| | **GPIO 33** | Rear-Right ESC Signal| PWM Output (LEDC / MCPWM)| Motor 4 Throttle Control |
| **Spare / Recommended**| **GPIO 13** | Active Buzzer / Status LED | Digital Output | Arming indicator or Lost Model alarm |
| | **GPIO 34** | LiPo Battery Voltage Divider | Analog Input (ADC1_CH6) | **Input-Only Pin**: 3S LiPo voltage monitoring |

### 2.2 Ground Station Receiver Pinout (XIAO ESP32S3 Sense)

The ground station receives telemetry from the drone using an SX1278 (Ra-02) LoRa module. The Seeed Studio XIAO ESP32S3 Sense is extremely compact, so we utilize its default hardware SPI pins alongside general-purpose digital pins for the LoRa module controls.

| Subsystem / Bus | XIAO Pin Label | ESP32-S3 GPIO | Target Module Pin | Description & Purpose |
| :--- | :--- | :--- | :--- | :--- |
| **Power** | **3V3** | - | SX1278 VCC | Provide 3.3V power (Use 100uF + 100nF decoupling capacitors) |
| | **GND** | - | SX1278 GND | Common Ground |
| **SPI Bus (LoRa)** | **D8** | **GPIO 7** | SX1278 SCK | Default Hardware SPI Clock |
| | **D9** | **GPIO 8** | SX1278 MISO | Default Hardware SPI MISO |
| | **D10** | **GPIO 9** | SX1278 MOSI | Default Hardware SPI MOSI |
| | **D3** | **GPIO 4** | SX1278 NSS / CS | Chip Select (Configurable in software) |
| | **D4** | **GPIO 5** | SX1278 RST | Hardware Reset (Configurable in software) |
| | **D5** | **GPIO 6** | SX1278 DIO0 | RxDone Interrupt (Configurable in software) |

*Note: The onboard camera and microphone on the Sense board use specific internal GPIOs, so using the standard exposed digital pins (D0-D10) is safe and avoids conflicts.*

---

## 3. Bus Architecture & Device Addresses

### 3.1 I2C Multi-Drop Bus Configuration
Three avionics sensors share the exact same I2C lines (`GPIO 21` and `GPIO 22`). They do not conflict because each device possesses a distinct 7-bit hardware address:

| Sensor | Default Address | Function | Alternate Address (Pin strapped) |
| :--- | :--- | :--- | :--- |
| **MPU-6050** | `0x68` | 6-DOF Rate Gyro & Accelerometer | `0x69` (if AD0 pulled HIGH) |
| **QMC5883L** | `0x0D` | 3-Axis Magnetometer / Compass | Fixed |
| **BMP388** | `0x76` | Barometric Pressure & Altitude | `0x77` (if SDO pulled HIGH) |

> [!NOTE]
> Ensure the BMP388 module SDO pin is tied to GND (or 3.3V) explicitly so its address remains deterministic (`0x76` or `0x77`).

---

## 4. Power Distribution & Electrical Architecture

```
                                      +---------------------------------------------+
                                      | 3S LiPo Battery (11.1V - 12.6V, 3300mAh)    |
                                      +----------------------+----------------------+
                                                             |
                                           [ XT60 Main Power Bus / PDB ]
                                                             |
                      +-------------------+------------------+-------------------+
                      |                   |                  |                   |
                  [ ESC 1 ]           [ ESC 2 ]          [ ESC 3 ]           [ ESC 4 ]
               (SimonK 30A)        (SimonK 30A)       (SimonK 30A)        (SimonK 30A)
                      |
              [ 5V 2A BEC Red Wire ]
                      |
                      +---> Powers ESP32 VIN (5V Pin)
                      +---> Powers NEO-6M GPS (5V VCC Pin)
                      
   +---------------------------------------------------------------------------------------+
   | ESP32 DevKit 3.3V Output Rail:                                                        |
   |   - MPU-6050 VCC (3.3V)                                                               |
   |   - QMC5883L VCC (3.3V)                                                               |
   |   - BMP388 VCC (3.3V)                                                                 |
   |   - LoRa SX1278 (Ra-02) VCC (3.3V) + [ 100uF + 100nF Decoupling Capacitors ]          |
   +---------------------------------------------------------------------------------------+
```

### 4.1 Critical Power Rules:
1. **Never power LoRa directly from weak pins**:
   During 433 MHz RF packet transmission at $+17\text{ dBm}$ to $+20\text{ dBm}$, the SX1278 can draw instantaneous current spikes between **$120\text{ mA}$ and $150\text{ mA}$**.
   - Place a **$100\,\mu\text{F}$ electrolytic capacitor** and a **$100\,\text{nF}$ ceramic capacitor** directly across the Ra-02 module's $3.3\text{V}$ and $\text{GND}$ pins.
   - Without these capacitors, the voltage sag causes ESP32 **Brownout Resets** mid-flight.
2. **ESC BEC Isolation (Single 5V Source)**:
   - SimonK 30A ESCs have internal linear 5V/2A BECs.
   - Connect the **Red (5V) wire of ONLY ONE ESC** to the ESP32 VIN / 5V rail.
   - On the remaining 3 ESCs, **cut, tape, or remove the red 5V pin** from the servo plug, keeping only the Signal and Ground wires. Connecting multiple linear regulators in parallel creates reverse currents and overheating.
3. **Common Ground is Mandatory**:
   - All grounds (LiPo ground, PDB, ESP32 GND, ESC signal GNDs, sensor GNDs, and LoRa GND) must be tied to a single unified common ground reference.

---

## 5. ESP32 Pin Pitfalls & What to Avoid

> [!WARNING]
> Respect the ESP32 hardware quirks below to prevent boot loops or destroyed hardware:

1. **Do NOT use GPIO 6, 7, 8, 9, 10, or 11**:
   These pins are connected internally to the SPI flash memory chip. Touching or wiring to them causes immediate ESP32 crashes.
2. **Do NOT use Input-Only Pins (GPIs) for Outputs or ESC Signals**:
   `GPIO 34, 35, 36 (VP), 39 (VN)` **do not have internal output drivers or pull-up/down resistors**. They can only be used as inputs (e.g., analog voltage sensors).
3. **Avoid Strapping Pins for Motors**:
   `GPIO 0, 2, 5, 12, 15` dictate boot mode (download mode vs. SPI boot) or toggle state during reboot. Connecting ESC signals to these pins can send unwanted high-frequency glitches to the motors during boot, causing the quadcopter to throttle up unexpectedly on power-up.

---

## 6. Avionics Safety & Flight Testing Protocols

### 6.1 Bench Testing Without Propellers (Rule #1)
> [!CAUTION]
> **NEVER ATTACH PROPELLERS INDOORS OR DURING CODE DEVELOPMENT!**  
> 1045 carbon fiber propellers on A2212 motors spinning at $11.1\text{V}$ can cause severe personal injury. Only mount propellers in an open field when failsafes and motor direction have been proven 100% reliable.

### 6.2 LoRa Watchdog & Failsafe Logic
- In drone flight, a lost radio packet is inevitable at distance.
- Implement an explicit **Failsafe Timeout**:
  ```cpp
  unsigned long lastTelemetryReceived = millis();
  const unsigned long FAILSAFE_TIMEOUT_MS = 1500; // 1.5 seconds

  void checkFailsafe() {
    if (millis() - lastTelemetryReceived > FAILSAFE_TIMEOUT_MS) {
      // Disarm motors or trigger gentle descent landing
      triggerMotorCutoff();
    }
  }
  ```

### 6.3 LiPo Battery Protection
- 3S LiPo nominal voltage is $11.1\text{V}$ (fully charged at $12.6\text{V}$, or $4.2\text{V}/\text{cell}$).
- **Absolute Minimum Cutoff:** Never discharge below **$3.3\text{V}/\text{cell}$ ($9.9\text{V}$ total)**.
- Connect a voltage divider ($100\,\text{k}\Omega$ and $22\,\text{k}\Omega$) from the battery rail to **GPIO 34** to read pack voltage in software and trigger an alarm before battery degradation occurs.

### 6.4 Mechanical Vibration Isolation
- The MPU-6050 accelerometer is extremely sensitive to high-frequency frame vibrations generated by the A2212 motors.
- Mount the ESP32 and sensor carrier board on silicone anti-vibration standoffs or thick double-sided foam tape on the center plate of the F450 frame.
- Ensure the QMC5883L compass is physically elevated away from the high-current PDB wires to prevent magnetic interference from high motor currents.
