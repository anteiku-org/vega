/*
 * ======================================================================================
 * PROJECT VEGA: LoRa Urban Benchmark & Parameter Test Suite (TRANSMITTER)
 * Target Hardware: ESP32 DevKit (36-Pin / 30-Pin)
 * Radio Module: SX1276 / Ra-02 (433MHz) on Hardware VSPI
 *
 * PINOUT:
 *   SCK  -> GPIO 18
 *   MISO -> GPIO 19
 *   MOSI -> GPIO 23
 *   NSS  -> GPIO 4 (or 5 depending on wiring)
 *   RST  -> GPIO 14
 *   DIO0 -> GPIO 26
 * ======================================================================================
 */

#include <SPI.h>
#include <LoRa.h>

// --- PIN ASSIGNMENTS (ESP32 DevKit) ---
#define LORA_SCK   18
#define LORA_MISO  19
#define LORA_MOSI  23
#define LORA_SS    4    // NSS / CS Pin
#define LORA_RST   14   // Reset Pin
#define LORA_DIO0  26   // Interrupt Pin

// --- TEST PROFILES ---
enum LoRaProfile {
    PROFILE_BASELINE = 1,        // Standard default settings
    PROFILE_URBAN_PENETRATION,   // SF10, BW 125k, CR 4/8, High Preamble (Cuts through walls)
    PROFILE_DRONE_FAST_CONTROL   // SF7, BW 250k, CR 4/5 (Low airtime for flight controls)
};

// >>> CHOOSE ACTIVE PROFILE HERE <<<
const LoRaProfile ACTIVE_PROFILE = PROFILE_URBAN_PENETRATION;

uint32_t packetCounter = 0;
unsigned long lastSendTime = 0;
uint32_t sendIntervalMs = 1000; // Send 1 packet every second

void applyProfile(LoRaProfile prof) {
    switch (prof) {
        case PROFILE_BASELINE:
            Serial.println("[PROFILE] Selected: 1. BASELINE (SF7, BW 125kHz, CR 4/5)");
            LoRa.setSignalBandwidth(125E3);
            LoRa.setSpreadingFactor(7);
            LoRa.setCodingRate4(5);
            LoRa.setPreambleLength(8);
            LoRa.setSyncWord(0x12);
            LoRa.setTxPower(17);
            sendIntervalMs = 500;
            break;

        case PROFILE_URBAN_PENETRATION:
            Serial.println("[PROFILE] Selected: 2. URBAN PENETRATION (SF10, BW 125kHz, CR 4/8, Off-channel)");
            // 433.175MHz shifts away from congested 433.000MHz car remotes
            LoRa.setFrequency(433.175E6);
            LoRa.setSignalBandwidth(125E3);
            LoRa.setSpreadingFactor(10);
            LoRa.setCodingRate4(8);        // 100% redundancy for noise immunity
            LoRa.setPreambleLength(12);    // Extra chirps to lock through building reflections
            LoRa.setSyncWord(0xF3);        // Custom private sync word
            LoRa.setTxPower(17);           // Max power (PA_BOOST)
            LoRa.enableCrc();
            sendIntervalMs = 1500;         // Slower rate due to higher airtime
            break;

        case PROFILE_DRONE_FAST_CONTROL:
            Serial.println("[PROFILE] Selected: 3. DRONE FAST CONTROL (SF7, BW 250kHz, CR 4/5)");
            LoRa.setFrequency(433.500E6);
            LoRa.setSignalBandwidth(250E3);
            LoRa.setSpreadingFactor(7);
            LoRa.setCodingRate4(5);
            LoRa.setPreambleLength(8);
            LoRa.setSyncWord(0xF3);
            LoRa.setTxPower(17);
            LoRa.enableCrc();
            sendIntervalMs = 100;          // 10Hz transmission rate
            break;
    }
}

void setup() {
    Serial.begin(115200);
    delay(1500);

    Serial.println("\n=======================================================");
    Serial.println("  PROJECT VEGA: LORA TRANSMITTER URBAN BENCHMARK");
    Serial.println("=======================================================");

    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
    LoRa.setSPI(SPI);
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

    if (!LoRa.begin(433E6)) {
        Serial.println("[ERROR] LoRa Initialization Failed!");
        Serial.println("        1. Check 3.3V Power & Common Ground");
        Serial.println("        2. Check NSS (GPIO 4), RST (14), DIO0 (26)");
        Serial.println("        3. Check SPI: SCK(18), MISO(19), MOSI(23)");
        while (1) { delay(1000); }
    }

    applyProfile(ACTIVE_PROFILE);
    Serial.println("[OK] LoRa Transmitter Ready! Sending benchmark packets...\n");
}

void loop() {
    unsigned long now = millis();
    if (now - lastSendTime >= sendIntervalMs) {
        lastSendTime = now;
        packetCounter++;

        // Benchmark payload: Counter + Timestamp + Dummy telemetry payload
        unsigned long txStart = micros();

        LoRa.beginPacket();
        LoRa.print("PKT:");
        LoRa.print(packetCounter);
        LoRa.print(",TIME:");
        LoRa.print(now);
        LoRa.print(",BAT:11.85V,ALT:14.2M");
        LoRa.endPacket(false); // Blocking transmit to calculate exact Airtime

        unsigned long airtimeUs = micros() - txStart;

        Serial.print("[TX] Sent #");
        Serial.print(packetCounter);
        Serial.print(" | Airtime: ");
        Serial.print(airtimeUs / 1000.0, 1);
        Serial.println(" ms");
    }
}
