/*
 * ======================================================================================
 * PROJECT VEGA: Full Avionics Suite Bench-Test & Demonstration Firmware
 * Microcontroller: ESP32 DevKit V1 (Dual-Core @ 240MHz)
 * Sensors Integrated:
 *   1. MPU-6050 (0x68): 6-DOF Rate Gyro & Accelerometer (Pitch & Roll Fusion)
 *   2. QMC5883L (0x0D): 3-Axis Digital Magnetometer (Electronic Compass
 * Heading)
 *   3. BMP280   (0x76): Barometric Pressure, Altitude & Temperature
 *   4. INA219   (0x40): Avionics & LiPo Battery Bus Voltage Monitor
 *   5. NEO-6M GPS (Serial2: RX=GPIO 16, TX=GPIO 17 @ 9600 Baud): Real-time NMEA
 * Telemetry
 *   6. Closed-Loop PID Controller & Quad-X Motor Mixer
 *   7. Standalone Wi-Fi Telemetry Web Server (SSID: "VEGA", Pass: "vega30")
 * ======================================================================================
 */

#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>

// --- Wi-Fi Access Point Configuration ---
// IMPORTANT: WPA2-PSK requires at least 8 characters! ("vega30" had only 6 chars, which caused ESP32 to fail!)
const char *ssid = "VEGA";
const char *password = "vegais30"; // Exactly 8 characters (Valid WPA2)
WebServer server(80);

TaskHandle_t WiFiTaskHandle;

// --- I2C Addresses ---
#define MPU_ADDR 0x68
#define QMC_ADDR 0x0D
#define BMP_ADDR                                                               \
  0x76 // Default BMP280 I2C address (fallback 0x77 checked at boot)
#define INA_ADDR 0x40

// --- Sensor Connection Flags ---
bool mpu_detected = false;
bool qmc_detected = false;
bool bmp_detected = false;
bool ina_detected = false;
bool gps_active = false;

// --- MPU-6050 Attitude & Gyro State ---
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float gyroX_cal = 0, gyroY_cal = 0, gyroZ_cal = 0;
float pitch = 0.0, roll = 0.0;
float pitch_acc = 0.0, roll_acc = 0.0;
unsigned long prev_time = 0;
float dt = 0.01;

// --- QMC5883L Compass State ---
float compass_heading = 0.0;
String compass_dir = "N";

// --- BMP280 Barometer State ---
float bmp_pressure = 1013.25; // hPa
float bmp_altitude = 0.0;     // Meters
float bmp_temp = 25.0;        // Celsius
float baseline_pressure = 1013.25;

// Calibration parameters for BMP280
uint16_t dig_T1 = 27504;
int16_t dig_T2 = 26435;
int16_t dig_T3 = -1000;
uint16_t dig_P1 = 36477;
int16_t dig_P2 = -10685;
int16_t dig_P3 = 3024;
int16_t dig_P4 = 2855;
int16_t dig_P5 = 140;
int16_t dig_P6 = -7;
int16_t dig_P7 = 15500;
int16_t dig_P8 = -14600;
int16_t dig_P9 = 6000;

// --- INA219 Power Monitor State ---
float lipo_voltage = 11.85; // Volts (3S LiPo default)

// --- GPS NEO-6M State ---
int gps_sats = 0;
String gps_status = "Searching Satellites...";
float gps_lat = 0.0;
float gps_lon = 0.0;
String nmea_buffer = "";

// --- PID Control Parameters ---
float desired_pitch = 0.0;
float desired_roll = 0.0;
float Kp_pitch = 3.5, Ki_pitch = 0.05, Kd_pitch = 1.2;
float Kp_roll = 3.5, Ki_roll = 0.05, Kd_roll = 1.2;

float error_pitch = 0, prev_error_pitch = 0, integral_pitch = 0, pid_pitch = 0;
float error_roll = 0, prev_error_roll = 0, integral_roll = 0, pid_roll = 0;

// --- Quad-X Motor Outputs (1000us - 2000us) ---
float base_throttle = 1450.0;
int motor1_pwm = 1000;
int motor2_pwm = 1000;
int motor3_pwm = 1000;
int motor4_pwm = 1000;

// -------------------------------------------------------------
// HTML / JS Telemetry Dashboard for Phone & Laptop
// -------------------------------------------------------------
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>PROJECT VEGA | Autonomous Flight Controller</title>
  <style>
    body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: #090d16; color: #f1f5f9; text-align: center; margin: 0; padding: 12px; }
    h1 { color: #38bdf8; margin: 5px 0; font-size: 22px; letter-spacing: 1px; }
    p.sub { color: #64748b; font-size: 12px; margin-top: 0; margin-bottom: 12px; text-transform: uppercase; letter-spacing: 1.5px; }
    
    .status-bar { display: flex; flex-wrap: wrap; justify-content: center; gap: 8px; margin-bottom: 15px; }
    .badge { font-size: 11px; font-weight: bold; padding: 4px 10px; border-radius: 20px; text-transform: uppercase; }
    .badge-on { background: #064e3b; color: #34d399; border: 1px solid #059669; }
    .badge-off { background: #450a0a; color: #f87171; border: 1px solid #dc2626; }

    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(140px, 1fr)); gap: 10px; max-width: 850px; margin: 0 auto; }
    .card { background: #131c2e; padding: 12px; border-radius: 10px; border: 1px solid #1e293b; box-shadow: 0 4px 8px rgba(0,0,0,0.4); }
    .label { font-size: 11px; color: #94a3b8; text-transform: uppercase; letter-spacing: 0.8px; margin-bottom: 4px; }
    .value { font-size: 24px; font-weight: 700; color: #38bdf8; }
    .unit { font-size: 14px; font-weight: normal; color: #94a3b8; }
    
    .section-title { color: #94a3b8; font-size: 12px; text-transform: uppercase; letter-spacing: 1px; margin: 20px 0 10px 0; }
    .motors { display: grid; grid-template-columns: repeat(4, 1fr); gap: 8px; max-width: 850px; margin: 0 auto; }
    .motor-box { background: #131c2e; border: 1px solid #1e293b; border-radius: 8px; padding: 10px 4px; }
    .bar-bg { background: #1e293b; height: 100px; width: 22px; border-radius: 4px; margin: 8px auto; position: relative; overflow: hidden; }
    .bar-fill { background: linear-gradient(to top, #0284c7, #38bdf8); width: 100%; position: absolute; bottom: 0; transition: height 0.08s ease; }
    .pwm-val { font-size: 12px; font-weight: bold; color: #f1f5f9; }
  </style>
</head>
<body>
  <h1>PROJECT VEGA &bull; AVIONICS SUITE</h1>
  <p class="sub">Autonomous Flight Controller Bench Telemetry</p>

  <div class="status-bar">
    <span class="badge" id="b_mpu">MPU-6050: CHECK</span>
    <span class="badge" id="b_qmc">QMC5883L: CHECK</span>
    <span class="badge" id="b_bmp">BMP280: CHECK</span>
    <span class="badge" id="b_ina">INA219: CHECK</span>
    <span class="badge" id="b_gps">GPS NEO-6M: CHECK</span>
  </div>

  <div class="grid">
    <div class="card">
      <div class="label">Pitch Attitude</div>
      <div class="value" id="val_pitch">0.0<span class="unit">&deg;</span></div>
    </div>
    <div class="card">
      <div class="label">Roll Attitude</div>
      <div class="value" id="val_roll">0.0<span class="unit">&deg;</span></div>
    </div>
    <div class="card">
      <div class="label">Compass Heading</div>
      <div class="value" id="val_compass">0<span class="unit">&deg;</span> <span style="font-size:16px; color:#34d399;" id="val_dir">N</span></div>
    </div>
    <div class="card">
      <div class="label">Relative Altitude</div>
      <div class="value" id="val_alt">0.0<span class="unit"> cm</span></div>
    </div>
    <div class="card">
      <div class="label">Baro Pressure</div>
      <div class="value" id="val_press">1013<span class="unit"> hPa</span></div>
    </div>
    <div class="card">
      <div class="label">Battery (INA219)</div>
      <div class="value" id="val_volt" style="color: #34d399;">11.8<span class="unit"> V</span></div>
    </div>
    <div class="card">
      <div class="label">Pitch PID Output</div>
      <div class="value" id="val_pid_pitch">0.0</div>
    </div>
    <div class="card">
      <div class="label">Roll PID Output</div>
      <div class="value" id="val_pid_roll">0.0</div>
    </div>
  </div>

  <div class="section-title">Navigation & GPS Satellite Tracking</div>
  <div class="grid" style="max-width: 850px;">
    <div class="card" style="grid-column: span 2;">
      <div class="label">GPS Status</div>
      <div class="value" style="font-size: 16px; margin: 4px 0;" id="val_gps_status">Searching...</div>
    </div>
    <div class="card">
      <div class="label">Satellites</div>
      <div class="value" id="val_gps_sats">0</div>
    </div>
  </div>

  <div class="section-title">Theoretical Motor Outputs (Quad-X Mixer)</div>
  <div class="motors">
    <div class="motor-box">
      <div class="label">M1 (RR)</div>
      <div class="bar-bg"><div class="bar-fill" id="bar_m1"></div></div>
      <div class="pwm-val" id="pwm_m1">1000 &micro;s</div>
    </div>
    <div class="motor-box">
      <div class="label">M2 (FR)</div>
      <div class="bar-bg"><div class="bar-fill" id="bar_m2"></div></div>
      <div class="pwm-val" id="pwm_m2">1000 &micro;s</div>
    </div>
    <div class="motor-box">
      <div class="label">M3 (RL)</div>
      <div class="bar-bg"><div class="bar-fill" id="bar_m3"></div></div>
      <div class="pwm-val" id="pwm_m3">1000 &micro;s</div>
    </div>
    <div class="motor-box">
      <div class="label">M4 (FL)</div>
      <div class="bar-bg"><div class="bar-fill" id="bar_m4"></div></div>
      <div class="pwm-val" id="pwm_m4">1000 &micro;s</div>
    </div>
  </div>

  <script>
    setInterval(() => {
      fetch('/data').then(res => res.json()).then(d => {
        document.getElementById('val_pitch').innerHTML = d.pitch.toFixed(1) + '<span class="unit">&deg;</span>';
        document.getElementById('val_roll').innerHTML = d.roll.toFixed(1) + '<span class="unit">&deg;</span>';
        document.getElementById('val_compass').innerHTML = d.heading.toFixed(0) + '<span class="unit">&deg;</span>';
        document.getElementById('val_dir').innerText = d.dir;
        document.getElementById('val_alt').innerHTML = (d.alt >= 0 ? "+" : "") + d.alt.toFixed(0) + '<span class="unit"> cm</span>';
        document.getElementById('val_press').innerHTML = d.press.toFixed(1) + '<span class="unit"> hPa</span>';
        document.getElementById('val_volt').innerHTML = d.volt.toFixed(2) + '<span class="unit"> V</span>';

        document.getElementById('val_pid_pitch').innerText = d.pid_p.toFixed(1);
        document.getElementById('val_pid_roll').innerText = d.pid_r.toFixed(1);

        document.getElementById('val_gps_status').innerText = d.gps_stat;
        document.getElementById('val_gps_sats').innerText = d.gps_sats;

        document.getElementById('pwm_m1').innerHTML = d.m1 + ' &micro;s';
        document.getElementById('pwm_m2').innerHTML = d.m2 + ' &micro;s';
        document.getElementById('pwm_m3').innerHTML = d.m3 + ' &micro;s';
        document.getElementById('pwm_m4').innerHTML = d.m4 + ' &micro;s';

        document.getElementById('bar_m1').style.height = ((d.m1 - 1000) / 10) + '%';
        document.getElementById('bar_m2').style.height = ((d.m2 - 1000) / 10) + '%';
        document.getElementById('bar_m3').style.height = ((d.m3 - 1000) / 10) + '%';
        document.getElementById('bar_m4').style.height = ((d.m4 - 1000) / 10) + '%';

        updateBadge('b_mpu', 'MPU-6050', d.s_mpu);
        updateBadge('b_qmc', 'QMC5883L', d.s_qmc);
        updateBadge('b_bmp', 'BMP280', d.s_bmp);
        updateBadge('b_ina', 'INA219', d.s_ina);
        updateBadge('b_gps', 'NEO-6M GPS', d.s_gps);
      }).catch(err => console.log(err));
    }, 80);

    function updateBadge(id, name, active) {
      const el = document.getElementById(id);
      if (active) {
        el.className = "badge badge-on";
        el.innerText = name + ": ACTIVE";
      } else {
        el.className = "badge badge-off";
        el.innerText = name + ": OFFLINE";
      }
    }
  </script>
</body>
</html>
)rawliteral";

// -------------------------------------------------------------
// Web Server Route Handlers
// -------------------------------------------------------------
void handleRoot() { server.send(200, "text/html", INDEX_HTML); }

void handleData() {
  String json = "{";
  json += "\"pitch\":" + String(pitch, 2) + ",";
  json += "\"roll\":" + String(roll, 2) + ",";
  json += "\"heading\":" + String(compass_heading, 1) + ",";
  json += "\"dir\":\"" + compass_dir + "\",";
  json += "\"alt\":" + String(bmp_altitude * 100.0, 1) + ",";
  json += "\"press\":" + String(bmp_pressure, 1) + ",";
  json += "\"volt\":" + String(lipo_voltage, 2) + ",";
  json += "\"pid_p\":" + String(pid_pitch, 2) + ",";
  json += "\"pid_r\":" + String(pid_roll, 2) + ",";
  json += "\"gps_stat\":\"" + gps_status + "\",";
  json += "\"gps_sats\":" + String(gps_sats) + ",";
  json += "\"m1\":" + String(motor1_pwm) + ",";
  json += "\"m2\":" + String(motor2_pwm) + ",";
  json += "\"m3\":" + String(motor3_pwm) + ",";
  json += "\"m4\":" + String(motor4_pwm) + ",";
  json += "\"s_mpu\":" + String(mpu_detected ? 1 : 0) + ",";
  json += "\"s_qmc\":" + String(qmc_detected ? 1 : 0) + ",";
  json += "\"s_bmp\":" + String(bmp_detected ? 1 : 0) + ",";
  json += "\"s_ina\":" + String(ina_detected ? 1 : 0) + ",";
  json += "\"s_gps\":" + String(gps_active ? 1 : 0);
  json += "}";
  server.send(200, "application/json", json);
}

// -------------------------------------------------------------
// SENSOR INITIALIZATION FUNCTIONS
// -------------------------------------------------------------

void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() == 0) {
    mpu_detected = true;
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00); // Wake up
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B);
    Wire.write(0x08); // Gyro ±500 deg/s
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C);
    Wire.write(0x10); // Accel ±8g
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1A);
    Wire.write(0x03); // Hardware DLPF 42Hz
    Wire.endTransmission(true);
    Serial.println("[OK] MPU-6050 Initialized (0x68)");
  } else {
    Serial.println("[FAIL] MPU-6050 Not Found!");
  }
}

void initQMC5883L() {
  Wire.beginTransmission(QMC_ADDR);
  if (Wire.endTransmission() == 0) {
    qmc_detected = true;
    // Set/Reset Period register (0x0B) = 0x01
    Wire.beginTransmission(QMC_ADDR);
    Wire.write(0x0B);
    Wire.write(0x01);
    Wire.endTransmission(true);

    // Control Register 1 (0x09): Continuous Mode, 200Hz, 8 Gauss, 512
    // Oversample = 0x1D
    Wire.beginTransmission(QMC_ADDR);
    Wire.write(0x09);
    Wire.write(0x1D);
    Wire.endTransmission(true);
    Serial.println("[OK] QMC5883L Magnetometer Initialized (0x0D)");
  } else {
    Serial.println("[INFO] QMC5883L Not Detected (0x0D)");
  }
}

void initBMP280() {
  Wire.beginTransmission(BMP_ADDR);
  if (Wire.endTransmission() == 0) {
    bmp_detected = true;
    // Read Trimming Calibration Coefficients from 0x88
    Wire.beginTransmission(BMP_ADDR);
    Wire.write(0x88);
    Wire.endTransmission(false);
    Wire.requestFrom(BMP_ADDR, 24, true);
    if (Wire.available() >= 24) {
      dig_T1 = Wire.read() | (Wire.read() << 8);
      dig_T2 = Wire.read() | (Wire.read() << 8);
      dig_T3 = Wire.read() | (Wire.read() << 8);
      dig_P1 = Wire.read() | (Wire.read() << 8);
      dig_P2 = Wire.read() | (Wire.read() << 8);
      dig_P3 = Wire.read() | (Wire.read() << 8);
      dig_P4 = Wire.read() | (Wire.read() << 8);
      dig_P5 = Wire.read() | (Wire.read() << 8);
      dig_P6 = Wire.read() | (Wire.read() << 8);
      dig_P7 = Wire.read() | (Wire.read() << 8);
      dig_P8 = Wire.read() | (Wire.read() << 8);
      dig_P9 = Wire.read() | (Wire.read() << 8);
    }
    // Ctrl_meas: Normal Mode, Temp x1, Press x1 = 0x27
    Wire.beginTransmission(BMP_ADDR);
    Wire.write(0xF4);
    Wire.write(0x27);
    Wire.endTransmission(true);

    // Config: Standby 1000ms, Filter 4 = 0xA0
    Wire.beginTransmission(BMP_ADDR);
    Wire.write(0xF5);
    Wire.write(0xA0);
    Wire.endTransmission(true);
    Serial.println("[OK] BMP280 Barometer Initialized (0x76)");
  } else {
    Serial.println("[INFO] BMP280 Not Detected (0x76)");
  }
}

void initINA219() {
  Wire.beginTransmission(INA_ADDR);
  if (Wire.endTransmission() == 0) {
    ina_detected = true;
    Serial.println("[OK] INA219 Voltage Monitor Initialized (0x40)");
  } else {
    Serial.println("[INFO] INA219 Not Detected (0x40)");
  }
}

// -------------------------------------------------------------
// SENSOR READING FUNCTIONS
// -------------------------------------------------------------

void readRawMPU() {
  if (!mpu_detected)
    return;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t rawAX = Wire.read() << 8 | Wire.read();
  int16_t rawAY = Wire.read() << 8 | Wire.read();
  int16_t rawAZ = Wire.read() << 8 | Wire.read();
  Wire.read();
  Wire.read(); // Skip temp
  int16_t rawGX = Wire.read() << 8 | Wire.read();
  int16_t rawGY = Wire.read() << 8 | Wire.read();
  int16_t rawGZ = Wire.read() << 8 | Wire.read();

  accX = (float)rawAX / 4096.0;
  accY = (float)rawAY / 4096.0;
  accZ = (float)rawAZ / 4096.0;

  gyroX = (float)rawGX / 65.5;
  gyroY = (float)rawGY / 65.5;
  gyroZ = (float)rawGZ / 65.5;
}

void readCompass() {
  if (!qmc_detected)
    return;
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(QMC_ADDR, 6, true);

  if (Wire.available() >= 6) {
    int16_t x = Wire.read() | (Wire.read() << 8);
    int16_t y = Wire.read() | (Wire.read() << 8);
    int16_t z = Wire.read() | (Wire.read() << 8);

    float heading = atan2((float)y, (float)x) * 180.0 / 3.141592;
    if (heading < 0)
      heading += 360.0;
    compass_heading = heading;

    // Convert to cardinal direction
    if (heading >= 337.5 || heading < 22.5)
      compass_dir = "N";
    else if (heading >= 22.5 && heading < 67.5)
      compass_dir = "NE";
    else if (heading >= 67.5 && heading < 112.5)
      compass_dir = "E";
    else if (heading >= 112.5 && heading < 157.5)
      compass_dir = "SE";
    else if (heading >= 157.5 && heading < 202.5)
      compass_dir = "S";
    else if (heading >= 202.5 && heading < 247.5)
      compass_dir = "SW";
    else if (heading >= 247.5 && heading < 292.5)
      compass_dir = "W";
    else
      compass_dir = "NW";
  }
}

void readBarometer() {
  if (!bmp_detected)
    return;
  Wire.beginTransmission(BMP_ADDR);
  Wire.write(0xF7);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP_ADDR, 6, true);

  if (Wire.available() >= 6) {
    int32_t adc_P = ((uint32_t)Wire.read() << 12) |
                    ((uint32_t)Wire.read() << 4) | ((uint32_t)Wire.read() >> 4);
    int32_t adc_T = ((uint32_t)Wire.read() << 12) |
                    ((uint32_t)Wire.read() << 4) | ((uint32_t)Wire.read() >> 4);

    // Temperature compensation
    int32_t var1_T =
        ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2_T = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
                        ((adc_T >> 4) - ((int32_t)dig_T1))) >>
                       12) *
                      ((int32_t)dig_T3)) >>
                     14;
    int32_t t_fine = var1_T + var2_T;
    bmp_temp = (float)((t_fine * 5 + 128) >> 8) / 100.0;

    // Pressure compensation
    int64_t var1_P = ((int64_t)t_fine) - 128000;
    int64_t var2_P = var1_P * var1_P * (int64_t)dig_P6;
    var2_P = var2_P + ((var1_P * (int64_t)dig_P5) << 17);
    var2_P = var2_P + (((int64_t)dig_P4) << 35);
    var1_P = ((var1_P * var1_P * (int64_t)dig_P3) >> 8) +
             ((var1_P * (int64_t)dig_P2) << 12);
    var1_P = (((((int64_t)1) << 47) + var1_P)) * ((int64_t)dig_P1) >> 33;

    if (var1_P != 0) {
      int64_t p = 1048576 - adc_P;
      p = (((p << 31) - var2_P) * 3125) / var1_P;
      var1_P = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
      var2_P = (((int64_t)dig_P8) * p) >> 19;
      p = ((p + var1_P + var2_P) >> 8) + (((int64_t)dig_P7) << 4);
      bmp_pressure = (float)p / 256.0 / 100.0; // hPa

      // Hypsometric formula for altitude in meters
      bmp_altitude =
          44330.0 * (1.0 - pow(bmp_pressure / baseline_pressure, 0.1903));
    }
  }
}

void readVoltage() {
  if (!ina_detected)
    return;
  Wire.beginTransmission(INA_ADDR);
  Wire.write(0x02); // Bus Voltage Register
  Wire.endTransmission(false);
  Wire.requestFrom(INA_ADDR, 2, true);

  if (Wire.available() >= 2) {
    uint16_t raw = (Wire.read() << 8) | Wire.read();
    lipo_voltage = (float)(raw >> 3) * 0.004; // 4mV per LSB
  }
}

unsigned long gps_bytes = 0;

void readGPS() {
  while (Serial2.available()) {
    char c = Serial2.read();
    gps_bytes++;
    gps_active = true;
    if (c == '\n') {
      if (nmea_buffer.indexOf("GGA") != -1) {
        // Parse Satellites in view from GGA sentence
        int commaIndex = 0;
        int satCountIndex = -1;
        for (int i = 0; i < nmea_buffer.length(); i++) {
          if (nmea_buffer[i] == ',') {
            commaIndex++;
            if (commaIndex == 7)
              satCountIndex = i + 1; // 7th field is satellites count
          }
        }
        if (satCountIndex != -1 && satCountIndex < nmea_buffer.length()) {
          gps_sats = nmea_buffer.substring(satCountIndex, satCountIndex + 2).toInt();
          if (gps_sats > 3) {
            gps_status = "3D Fix (" + String(gps_sats) + " Sats)";
          } else {
            gps_status = "Indoor Lock (" + String(gps_sats) + " Sats in view)";
          }
        }
      }
      nmea_buffer = "";
    } else if (c != '\r') {
      if (nmea_buffer.length() < 100) nmea_buffer += c;
    }
  }

  // Diagnostic feedback for user & professor
  if (gps_bytes > 0 && gps_sats == 0) {
    gps_status = "NMEA Stream Active (" + String(gps_bytes) + " bytes)";
  } else if (gps_bytes == 0) {
    gps_status = "No UART Data (Check GPS TX->16, RX->17)";
  }
}

// -------------------------------------------------------------
// FREERTOS TASK FOR CORE 0: DEDICATED WI-FI & WEB SERVER
// -------------------------------------------------------------
void wifiTask(void *pvParameters) {
  Serial.print("[RTOS] Wi-Fi Server Task running on Core: ");
  Serial.println(xPortGetCoreID());
  while (true) {
    server.handleClient();
    vTaskDelay(5 / portTICK_PERIOD_MS); // 5ms sleep to yield CPU
  }
}

// -------------------------------------------------------------
// SETUP ROUTINE
// -------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(
      "\n=== [PROJECT VEGA: INTEGRATED AVIONICS INITIALIZATION] ===");

  // 1. Hardware Serial 2 for GPS (RX2=16, TX2=17 @ 9600 Baud)
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println(
      "[OK] Hardware Serial2 (GPS) started on GPIO 16/17 @ 9600 baud");

  // 2. Initialize Fast I2C Bus (SDA=21, SCL=22 @ 400kHz)
  Wire.begin(21, 22, 400000);

  // 3. Detect and Initialize All Sensors
  initMPU();
  initQMC5883L();
  initBMP280();
  initINA219();

  // 4. Calibrate Gyroscope Offsets (Keep breadboard flat for 2 seconds)
  if (mpu_detected) {
    Serial.println("Calibrating Gyroscope Offsets (KEEP BREADBOARD STILL)...");
    for (int i = 0; i < 400; i++) {
      readRawMPU();
      gyroX_cal += gyroX;
      gyroY_cal += gyroY;
      gyroZ_cal += gyroZ;
      delay(4);
    }
    gyroX_cal /= 400.0;
    gyroY_cal /= 400.0;
    gyroZ_cal /= 400.0;
    Serial.println("[OK] Gyroscope Calibrated Successfully!");
  }

  // Set baseline pressure for relative altitude
  if (bmp_detected) {
    delay(50);
    readBarometer();
    if (bmp_pressure > 300.0)
      baseline_pressure = bmp_pressure;
  }

  // 5. Start Wi-Fi Access Point (Forcibly clears old Wi-Fi memory)
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(100);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.print("Wi-Fi Hotspot Started! Connect to: ");
  Serial.println(ssid);
  Serial.print("Dashboard URL: http://");
  Serial.println(WiFi.softAPIP());

  // 6. Start Web Server
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();

  // 7. Pin Wi-Fi Web Server to Core 0 (Leaves Core 1 100% dedicated to Flight Control!)
  xTaskCreatePinnedToCore(
      wifiTask,         // Task function
      "WiFiWebServer",  // Task name
      4096,             // Stack size
      NULL,             // Parameters
      1,                // Priority
      &WiFiTaskHandle,  // Task handle
      0                 // CORE 0!
  );

  prev_time = micros();
}

// -------------------------------------------------------------
// MAIN CONTROL LOOP (Runs on CORE 1 by default at 100Hz!)
// -------------------------------------------------------------
void loop() {
  readGPS();

  unsigned long current_time = micros();
  dt = (float)(current_time - prev_time) / 1000000.0;
  if (dt < 0.010)
    return; // 100Hz deterministic rate
  prev_time = current_time;

  // 1. Read MPU-6050 & Compute Attitude Fusion
  if (mpu_detected) {
    readRawMPU();
    float gx_corrected = gyroX - gyroX_cal;
    float gy_corrected = gyroY - gyroY_cal;

    pitch_acc = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 57.2958;
    roll_acc = atan2(accY, accZ) * 57.2958;

    pitch = 0.98 * (pitch + gy_corrected * dt) + 0.02 * pitch_acc;
    roll = 0.98 * (roll + gx_corrected * dt) + 0.02 * roll_acc;
  }

  // 2. Closed-Loop PID Calculations (Pitch Axis)
  error_pitch = desired_pitch - pitch;
  integral_pitch += error_pitch * dt;
  integral_pitch = constrain(integral_pitch, -50.0, 50.0);
  float derivative_pitch = (error_pitch - prev_error_pitch) / dt;
  pid_pitch = (Kp_pitch * error_pitch) + (Ki_pitch * integral_pitch) +
              (Kd_pitch * derivative_pitch);
  prev_error_pitch = error_pitch;

  // 3. Closed-Loop PID Calculations (Roll Axis)
  error_roll = desired_roll - roll;
  integral_roll += error_roll * dt;
  integral_roll = constrain(integral_roll, -50.0, 50.0);
  float derivative_roll = (error_roll - prev_error_roll) / dt;
  pid_roll = (Kp_roll * error_roll) + (Ki_roll * integral_roll) +
             (Kd_roll * derivative_roll);
  prev_error_roll = error_roll;

  // 4. Quad-X Motor Mixer
  motor1_pwm = constrain(base_throttle - pid_pitch + pid_roll, 1000, 2000);
  motor2_pwm = constrain(base_throttle + pid_pitch + pid_roll, 1000, 2000);
  motor3_pwm = constrain(base_throttle - pid_pitch - pid_roll, 1000, 2000);
  motor4_pwm = constrain(base_throttle + pid_pitch - pid_roll, 1000, 2000);

  // 5. Sample Lower-Frequency Sensors
  static int counter = 0;
  counter++;
  if (counter % 5 == 0) { // 20Hz: Compass & Barometer
    readCompass();
    readBarometer();
  }
  if (counter % 20 == 0) { // 5Hz: Voltage
    readVoltage();
  }

  // 6. Serial Plotter Output (Clean comma-separated format for Arduino IDE 2.x)
  Serial.print("Pitch:");   Serial.print(pitch, 1);
  Serial.print(",");
  Serial.print("Roll:");    Serial.print(roll, 1);
  Serial.print(",");
  Serial.print("M1_RR:");   Serial.print(motor1_pwm);
  Serial.print(",");
  Serial.print("M2_FR:");   Serial.print(motor2_pwm);
  Serial.print(",");
  Serial.print("M3_RL:");   Serial.print(motor3_pwm);
  Serial.print(",");
  Serial.print("M4_FL:");   Serial.println(motor4_pwm);
}
