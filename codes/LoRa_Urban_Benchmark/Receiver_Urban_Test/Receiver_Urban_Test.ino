/*
 * ======================================================================================
 * PROJECT VEGA: LoRa Urban Benchmark & Parameter Test Suite (RECEIVER)
 * Target Hardware: Standard ESP32 DevKit OR Seeed Studio XIAO ESP32S3
 * Radio Module: SX1276 / Ra-02 (433MHz)
 *
 * BUZZER FEATURE: "Geiger-Counter" Audible RSSI Beacon
 *   - Strong Signal (LOS / Close by)  -> Rapid, fast beeping (120ms - 250ms interval)
 *   - Moderate Signal (Through walls)  -> Medium beeping (550ms interval)
 *   - Weak Signal (Edge of range)      -> Slow beeping (1100ms - 2000ms interval)
 *   - Lost Signal (>3.5s no packets)   -> Long warning pulse every 4 seconds
 * ======================================================================================
 */

#include <SPI.h>
#include <LoRa.h>

// --- AUTO-DETECT BOARD PINOUT ---
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_XIAO_ESP32S3)
    // Seeed Studio XIAO ESP32S3 Sense Pinout
    #define BOARD_NAME  "XIAO ESP32-S3 Sense"
    #define LORA_SCK    7   // D8
    #define LORA_MISO   8   // D9
    #define LORA_MOSI   9   // D10
    #define LORA_SS     4   // D3
    #define LORA_RST    5   // D4
    #define LORA_DIO0   6   // D5
    #define SENSE_SD_CS 21  // Disable onboard SD card
    #define PIN_BUZZER  2   // XIAO D1 (GPIO 2)
    SPIClass customSPI(FSPI);
    #define USE_CUSTOM_SPI 1
#else
    // Standard ESP32 DevKit (30-Pin / 36-Pin / 38-Pin)
    #define BOARD_NAME  "Standard ESP32 DevKit"
    #define LORA_SCK    18  // VSPI Clock
    #define LORA_MISO   19  // VSPI MISO
    #define LORA_MOSI   23  // VSPI MOSI
    #define LORA_SS     4   // NSS / CS
    #define LORA_RST    14  // Reset
    #define LORA_DIO0   26  // DIO0 Interrupt
    #define PIN_BUZZER  13  // Standard ESP32 GPIO 13
    #define USE_CUSTOM_SPI 0
#endif

// --- TEST PROFILES (MUST MATCH TRANSMITTER!) ---
enum LoRaProfile {
    PROFILE_BASELINE = 1,
    PROFILE_URBAN_PENETRATION,
    PROFILE_DRONE_FAST_CONTROL
};

// >>> MUST MATCH THE TRANSMITTER PROFILE! <<<
const LoRaProfile ACTIVE_PROFILE = PROFILE_URBAN_PENETRATION;

// Diagnostic Metrics
uint32_t totalPacketsReceived = 0;
uint32_t lastSequenceReceived = 0;
uint32_t packetsLost = 0;

// Signal Tracking for Buzzer
int lastRssi = -120;
float lastSnr = -10.0;
unsigned long lastPacketReceivedTime = 0;

// Non-blocking Buzzer State
bool buzzerActive = false;
unsigned long beepStartTime = 0;
unsigned long lastBeepCycleStart = 0;

void applyProfile(LoRaProfile prof) {
    switch (prof) {
        case PROFILE_BASELINE:
            Serial.println("[PROFILE] Selected: 1. BASELINE (SF7, BW 125kHz, CR 4/5)");
            LoRa.setSignalBandwidth(125E3);
            LoRa.setSpreadingFactor(7);
            LoRa.setCodingRate4(5);
            LoRa.setPreambleLength(8);
            LoRa.setSyncWord(0x12);
            break;

        case PROFILE_URBAN_PENETRATION:
            Serial.println("[PROFILE] Selected: 2. URBAN PENETRATION (SF10, BW 125kHz, CR 4/8, Off-channel)");
            LoRa.setFrequency(433.175E6);
            LoRa.setSignalBandwidth(125E3);
            LoRa.setSpreadingFactor(10);
            LoRa.setCodingRate4(8);
            LoRa.setPreambleLength(12);
            LoRa.setSyncWord(0xF3);
            LoRa.enableCrc();
            break;

        case PROFILE_DRONE_FAST_CONTROL:
            Serial.println("[PROFILE] Selected: 3. DRONE FAST CONTROL (SF7, BW 250kHz, CR 4/5)");
            LoRa.setFrequency(433.500E6);
            LoRa.setSignalBandwidth(250E3);
            LoRa.setSpreadingFactor(7);
            LoRa.setCodingRate4(5);
            LoRa.setPreambleLength(8);
            LoRa.setSyncWord(0xF3);
            LoRa.enableCrc();
            break;
    }
}

String getSignalQuality(int rssi, float snr) {
    if (snr > 5.0 && rssi > -80)  return "EXCELLENT (LOS)";
    if (snr > 0.0 && rssi > -100) return "GOOD (Moderate Obstacles)";
    if (snr > -10.0 && rssi > -115) return "DEGRADED (Heavy Concrete / Walls)";
    return "CRITICAL / EDGE-OF-RANGE";
}

// -------------------------------------------------------------
// NON-BLOCKING GEIGER-STYLE BUZZER ROUTINE
// -------------------------------------------------------------
void updateGeigerBuzzer() {
    unsigned long now = millis();
    bool signalLost = (now - lastPacketReceivedTime > 3500);

    uint32_t intervalMs = 2000;
    uint32_t beepDurationMs = 30; // 30ms crisp chirp

    if (signalLost) {
        intervalMs = 4000;       // Slow 4-second pulse
        beepDurationMs = 150;    // Long warning tone for lost signal
    } else {
        // Dynamic interval mapped to RSSI
        if (lastRssi >= -70)       intervalMs = 120;  // Super fast (Strong LOS)
        else if (lastRssi >= -85)  intervalMs = 250;  // Fast
        else if (lastRssi >= -100) intervalMs = 550;  // Medium (Through walls)
        else if (lastRssi >= -112) intervalMs = 1100; // Slow (Weak)
        else                       intervalMs = 2000; // Very slow (Edge of reception)
    }

    if (buzzerActive) {
        if (now - beepStartTime >= beepDurationMs) {
            digitalWrite(PIN_BUZZER, LOW);
            buzzerActive = false;
            lastBeepCycleStart = now;
        }
    } else {
        if (now - lastBeepCycleStart >= intervalMs) {
            digitalWrite(PIN_BUZZER, HIGH);
            buzzerActive = true;
            beepStartTime = now;
        }
    }
}

void setup() {
#if defined(SENSE_SD_CS)
    pinMode(SENSE_SD_CS, OUTPUT);
    digitalWrite(SENSE_SD_CS, HIGH);
#endif

    // Initialize Buzzer Pin
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);

    Serial.begin(115200);
    delay(1500);

    Serial.println("\n=======================================================");
    Serial.println("  PROJECT VEGA: LORA RECEIVER URBAN BENCHMARK & METRICS");
    Serial.print("  Detected Hardware: ");
    Serial.println(BOARD_NAME);
    Serial.print("  Geiger Buzzer Output: GPIO ");
    Serial.println(PIN_BUZZER);
    Serial.println("=======================================================");

#if USE_CUSTOM_SPI
    customSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
    LoRa.setSPI(customSPI);
#else
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
    LoRa.setSPI(SPI);
#endif

    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

    if (!LoRa.begin(433E6)) {
        Serial.println("[ERROR] LoRa Receiver Initialization Failed!");
        Serial.println("        1. Check 3.3V power & common GND");
        Serial.println("        2. Check NSS/CS, RST, DIO0, and SPI wiring");
        while (1) { delay(1000); }
    }

    applyProfile(ACTIVE_PROFILE);
    Serial.println("[OK] LoRa Receiver Active! Waiting for packets...\n");

    // Startup confirmation chirp
    digitalWrite(PIN_BUZZER, HIGH);
    delay(80);
    digitalWrite(PIN_BUZZER, LOW);
}

void loop() {
    // 1. Run Non-blocking Geiger Buzzer (Ticks continuously)
    updateGeigerBuzzer();

    // 2. Check for incoming LoRa Packets
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        String payload = "";
        while (LoRa.available()) {
            payload += (char)LoRa.read();
        }

        totalPacketsReceived++;
        lastRssi = LoRa.packetRssi();
        lastSnr = LoRa.packetSnr();
        lastPacketReceivedTime = millis();

        // Extract packet sequence number (Payload format: "PKT:<num>,...")
        uint32_t currentSeq = 0;
        int pktIdx = payload.indexOf("PKT:");
        int commaIdx = payload.indexOf(",", pktIdx);
        if (pktIdx != -1 && commaIdx != -1) {
            currentSeq = payload.substring(pktIdx + 4, commaIdx).toInt();
        }

        // Detect dropped / lost packets
        if (lastSequenceReceived > 0 && currentSeq > (lastSequenceReceived + 1)) {
            uint32_t missed = currentSeq - lastSequenceReceived - 1;
            packetsLost += missed;
            Serial.print(">>> [WARNING] MISSED ");
            Serial.print(missed);
            Serial.println(" PACKET(S)! <<<");
        }
        lastSequenceReceived = currentSeq;

        float packetLossRate = 0.0;
        if (totalPacketsReceived + packetsLost > 0) {
            packetLossRate = ((float)packetsLost / (float)(totalPacketsReceived + packetsLost)) * 100.0f;
        }

        // Print Diagnostic Telemetry
        Serial.print("[RX #");
        Serial.print(currentSeq);
        Serial.print("] Payload: '");
        Serial.print(payload);
        Serial.print("' | RSSI: ");
        Serial.print(lastRssi);
        Serial.print(" dBm | SNR: ");
        Serial.print(lastSnr, 1);
        Serial.print(" dB | Loss: ");
        Serial.print(packetLossRate, 1);
        Serial.print("% | Link: ");
        Serial.println(getSignalQuality(lastRssi, lastSnr));
    }
}
