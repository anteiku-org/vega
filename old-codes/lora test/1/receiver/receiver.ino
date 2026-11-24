#include <SPI.h>
#include <LoRa.h>

// Seeed Studio XIAO ESP32S3 Sense LoRa Pins
// We use raw GPIO numbers to bypass any board-manager macro bugs.
// XIAO D8 = GPIO 7
// XIAO D9 = GPIO 8
// XIAO D10 = GPIO 9
// XIAO D3 = GPIO 4
// XIAO D4 = GPIO 5
// XIAO D5 = GPIO 6
#define LORA_SCK   7
#define LORA_MISO  8
#define LORA_MOSI  9
#define LORA_SS    4
#define LORA_RST   5
#define LORA_DIO0  6

// The XIAO ESP32S3 Sense expansion board has an SD card sharing the SPI bus.
// We MUST pull its Chip Select (GPIO 21) HIGH to prevent it from corrupting the LoRa signals.
#define SENSE_SD_CS 21 

// Create a custom, isolated SPI bus instance (FSPI is standard for user SPI on S3)
SPIClass customSPI(FSPI);

void setup() {
  // 1. Disable the SD card immediately!
  pinMode(SENSE_SD_CS, OUTPUT);
  digitalWrite(SENSE_SD_CS, HIGH);

  Serial.begin(115200);
  delay(2000); 

  Serial.println("\n=================================");
  Serial.println("XIAO ESP32S3 Booted! Starting LoRa Receiver...");
  Serial.println("=================================");

  // 2. Explicitly start our isolated SPI bus with XIAO specific pins
  customSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  
  // 3. Tell the LoRa library to strictly use this custom SPI bus
  LoRa.setSPI(customSPI);

  // 4. Override the default CS, reset, and IRQ pins
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  // Must match transmitter frequency (433 MHz for Ra-02)
  if (!LoRa.begin(433E6)) {
    Serial.println("[-] Starting LoRa failed!");
    Serial.println("    Check 3.3V power, GND, NSS(D3), and SPI wiring (D8, D9, D10).");
    while (1) {
      delay(500);
    }
  }

  Serial.println("[+] LoRa Ra-02 Receiver Ready on 433 MHz!");
}

void loop() {
  int packetSize = LoRa.parsePacket();

  if (packetSize) {
    Serial.print("Received packet: '");

    while (LoRa.available()) {
      String LoRaData = LoRa.readString();
      Serial.print(LoRaData);
    }

    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
}