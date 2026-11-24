# Custom ESP32 Flight Controller PCB Design & Wiring Master Guide

**Project:** Autonomous Flight Controller with Dual-Redundant IMU & LoRa Telemetry  
**Microcontroller:** ESP32 DevKit V1 (30-pin DOIT form factor)  
**Form Factor:** Through-hole / Female Socket Headers for modular sensor replacement  

---

## 1. Dual-MPU6050 Redundancy Architecture (G1 & G2)

Having through-hole footprints for **two MPU6050 modules** on your PCB is a genuine aerospace/defense (DRDO-grade) technique known as **Sensor Redundancy**.

Both sensors share the exact same I2C lines (`SDA: GPIO 21`, `SCL: GPIO 22`), but are differentiated using the hardware `AD0` address pin:

| Sensor Header | Role | AD0 Pin Hardwired To | I2C Address | Initial Assembly |
| :--- | :--- | :--- | :--- | :--- |
| **G1 (Primary)** | Primary Flight IMU | **GND** | `0x68` | Populated with MPU-6050 |
| **G2 (Secondary)**| Redundant Backup IMU | **3.3V** | `0x69` | Unpopulated (Empty female header for future) |

*When G2 is empty, the bus functions with single MPU. When you plug in G2 later, the firmware can cross-check readings and trigger automatic failover if G1 locks up!*

---

## 2. Complete Schematic Pin-to-Pin Master Netlist

### 2.1 Shared I2C Bus (Sensors)
All sensors connect to **3.3V** and common **GND**.

| Component / Pin | Connects to Net | Target Pin on ESP32 | Notes |
| :--- | :--- | :--- | :--- |
| **G1 (MPU 1) SCL** | `I2C_SCL` | **GPIO 22** | Hardware I2C Clock |
| **G1 (MPU 1) SDA** | `I2C_SDA` | **GPIO 21** | Hardware I2C Data |
| **G1 (MPU 1) AD0** | `GND` | **GND** | Forces Address `0x68` |
| **G2 (MPU 2) SCL** | `I2C_SCL` | **GPIO 22** | Shared Clock |
| **G2 (MPU 2) SDA** | `I2C_SDA` | **GPIO 21** | Shared Data |
| **G2 (MPU 2) AD0** | `+3.3V` | **3.3V** | Forces Address `0x69` |
| **Q1 (QMC5883L) SCL**| `I2C_SCL` | **GPIO 22** | Shared Clock |
| **Q1 (QMC5883L) SDA**| `I2C_SDA` | **GPIO 21** | Shared Data |
| **U2 (BMP280) SCL** | `I2C_SCL` | **GPIO 22** | Shared Clock |
| **U2 (BMP280) SDA** | `I2C_SDA` | **GPIO 21** | Shared Data |
| **U2 (BMP280) CSB** | `+3.3V` | **3.3V** | **Crucial:** Forces BMP280 into I2C mode |
| **P1 (INA219) VCC** | `+3.3V` | **3.3V** | Powers internal logic (keeps I2C at 3.3V) |
| **P1 (INA219) GND** | `GND` | **GND** | Reference ground |
| **P1 (INA219) SCL** | `I2C_SCL` | **GPIO 22** | Shared Clock |
| **P1 (INA219) SDA** | `I2C_SDA` | **GPIO 21** | Shared Data |
| **P1 (INA219) VIN+**| `BATT_SENSE`| **LiPo Battery (+)** | Measures 3S/4S battery voltage |
| **P1 (INA219) VIN-**| `BATT_SENSE`| **Short to VIN+** | Prevents unused shunt resistor from floating |

---

### 2.2 Telemetry & Navigation (LoRa & GPS)

#### L1: LoRa SX1278 (Hardware VSPI)
| LoRa Pin | Connects to ESP32 Pin | Function |
| :--- | :--- | :--- |
| **VCC (3V3)** | `+3.3V` Rail | Decoupled with C2 (100uF) |
| **GND** (1, 2, 9, 16)| `GND` Plane | Connect all ground pins |
| **SCK (Pin 12)** | **GPIO 18** | VSPI Clock |
| **MISO (Pin 13)** | **GPIO 19** | VSPI MISO |
| **MOSI (Pin 14)** | **GPIO 23** | VSPI MOSI |
| **NSS / CS (Pin 15)**| **GPIO 5** | Chip Select |
| **RESET (Pin 4)** | **GPIO 14** | Radio Hardware Reset |
| **DIO0 (Pin 5)** | **GPIO 26** | RxDone / TxDone Interrupt line |

#### U3: GPS NEO-6M (Hardware Serial2 / UART)
| GPS Pin | Connects to | Notes |
| :--- | :--- | :--- |
| **VCC (Pin 1)** | `+5V` Rail | NEO-6M has onboard 3.3V regulator; feed it 5V |
| **RX (Pin 2)** | **ESP32 TX2 (GPIO 17)** | ESP32 commands to GPS |
| **TX (Pin 3)** | **ESP32 RX2 (GPIO 16)** | NMEA sentence stream into ESP32 |
| **GND (Pin 4)** | `GND` Plane | Ground |

---

### 2.3 Motor Actuation (ESCs H1 to H4)
Glitch-free, boot-safe PWM outputs:

| ESC Connector | Signal Pin | ESP32 GPIO | Quadcopter Quadrant |
| :--- | :--- | :--- | :--- |
| **H1 (ESC 1)** | SIG | **GPIO 32** | Rear-Right (Motor 1) |
| **H2 (ESC 2)** | SIG | **GPIO 33** | Front-Right (Motor 2) |
| **H3 (ESC 3)** | SIG | **GPIO 25** | Rear-Left (Motor 3) |
| **H1–H4 5V** | 5V | **NC (No Connection)** | **Leave Unconnected!** Avoids backfeed/conflict with 5V Buck converter. |
| **H1–H4 GND** | GND | `GND` Plane | Signal ground reference (Connect to common GND) |

---

### 2.4 Power, Switch (`KEY1`) & Decoupling Capacitors
* **External 5V Buck Converter Output** $\rightarrow$ **Switch KEY1 (Pin 1)**
* **Switch KEY1 (Pin 2)** $\rightarrow$ **ESP32 `VIN` Pin** & **GPS `VCC`**
  * *When KEY1 is OFF: You can plug in USB to program the ESP32 without backfeeding the buck converter.*
* **Capacitor C1 (100µF / 16V):** Solder across `+5V` and `GND` near the ESP32 VIN pin.
* **Capacitor C2 (100µF to 470µF / 10V):** Solder across `+3.3V` and `GND` directly beside the **LoRa module** to smooth TX current pulses.

#### INA219 Battery Voltage Sensing (Safe / Low-Current Path):
* **VIN+ (Pins 6 & 8):** Connect directly to **LiPo Battery (+) Positive Terminal**.
* **VIN- (Pins 5 & 7):** Connect directly to **VIN+** (shorted together on PCB).
  * *Why:* Because you are only measuring battery voltage (not current), zero heavy current travels through the INA219 or your PCB. Standard thin PCB traces (0.25mm / 10mil) are completely safe, with zero heat generated.

---

## 3. High-Value Additions to Include Before PCB Manufacturing

Since you are fabricating this board, adding these few components now will save you from redesigning later:

### 1. Active Buzzer Header (GPIO 13)
* **What to add:** A 2-pin header labeled `BUZZER` (GPIO 13 and GND).
* **Why:** In real drone flights, a lost-model alarm is mandatory. If your drone crashes into tall grass or trees, you toggle a switch on your controller and the buzzer beeps continuously so you can find it. It also beeps on arming/disarming.

### 2. External RC Receiver Port (UART / SBUS)
* **What to add:** A 3-pin header (`5V`, `GND`, `RX1 / GPIO 4`).
* **Why:** If you ever want to fly with a standard RC transmitter (FlySky, Radiomaster ELRS, TBS Crossfire) instead of purely automated LoRa commands, having this 3-pin header ready is a lifesaver.

### 3. Drone Frame Mounting Holes (30.5 × 30.5 mm)
* **What to add:** 4 unplated M3 holes placed in a square grid of **30.5 mm × 30.5 mm** (the international standard for quadcopter flight controller stacks).
* **Why:** This lets you mount the PCB onto standard carbon-fiber drone frames using anti-vibration rubber standoffs (bobbins).

### 4. I2C Bus Pull-up Resistor Footprints (4.7 kΩ)
* **What to add:** Solder pads for two 4.7 kΩ resistors: one between `SDA` & `3.3V`, and one between `SCL` & `3.3V`.
* **Why:** With 4 sensors on one I2C bus (MPU, BMP, QMC, INA), the bus capacitance increases. Having strong pull-up resistors keeps the square wave signals sharp at 400 kHz.

### 5. Ground Plane (Copper Pour)
* In your PCB layout editor (EasyEDA/KiCad), fill both the **Top Layer** and **Bottom Layer** with a `GND` copper pour.
* **Why:** Brushless motors and ESCs generate massive electromagnetic interference (EMI). A continuous ground plane shields your sensitive MPU6050 and magnetometer from motor noise.
