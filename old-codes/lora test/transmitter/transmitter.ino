#include <SPI.h>
#include <LoRa.h>

// ESP32 Pins for Ra-02 (SX1278)
#define SS_PIN    4    // NSS / CS
#define RST_PIN   14   // RST
#define DIO0_PIN  26   // DIO0

int counter = 0;

void setup() {
  Serial.begin(115200);x
  delay(1000);

  Serial.println("\n=================================");
  Serial.println("ESP32 Booted! Starting LoRa Sender...");
  Serial.println("=================================");

  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);

  if (!LoRa.begin(433E6)) {
    Serial.println("[-] Starting LoRa failed!");
    Serial.println("    Check 3.3V power, GND, NSS(4), and SPI wiring (18, 19, 23).");
    while (1) {
      delay(500);
    }
  }

  // Set transmission power (range 2 to 20 dBm, 17 is safe & strong for Ra-02)
  LoRa.setTxPower(17);

  Serial.println("[+] LoRa Ra-02 Initialized Successfully on 433 MHz!");
}

void loop() {
  Serial.print("Sending packet: ");
  Serial.println(counter);

  // Send packet using async mode (false = blocking, true = non-blocking)
  // We use async so bad DIO0 wiring won't freeze the ESP32!
  LoRa.beginPacket();
  LoRa.print("Hello from Ra-02! Packet #");
  LoRa.print(counter);
  LoRa.endPacket(true); // <--- non-blocking transmit!

  Serial.println("Packet sent to radio queue!");

  counter++;
  delay(2000);
}