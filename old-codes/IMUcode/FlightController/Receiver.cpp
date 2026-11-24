#include "Receiver.h"
#include "pins.h"
#include <SPI.h>
#include <RF24.h>

// NRF24L01 on ESP32 (MOSI=26, MISO=19, SCK=18, CSN=5, CE=4)
RF24 radio(NRF_CE, NRF_CSN);
const byte address[6] = "ABCDE";

RadioData rxData;
unsigned long lastRxTime = 0;

// Internal variables that will hold the final mapped values
int mapped_throttle = 1000;
float mapped_roll = 0.0;
float mapped_pitch = 0.0;
float mapped_yaw = 0.0;

void Receiver_init() {
    // Initialize default safe values
    rxData.throttle = 1000;
    rxData.pitch = 1500;
    rxData.roll = 1500;
    rxData.yaw = 1500;
    rxData.arm = 0;
    
    // Initialize NRF24L01 using custom SPI pins to avoid ESC conflicts!
    // SCK = 18, MISO = 19, MOSI = 26, SS/CSN = 5
    SPI.begin(18, 19, 26, 5);
    
    if (radio.begin()) {
        Serial.println("NRF24L01 Connected.");
    } else {
        Serial.println("NRF24L01 NOT FOUND! Check wiring.");
    }
    
    // Match the robust settings exactly
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_1MBPS);
    radio.setChannel(108);
    radio.setPayloadSize(sizeof(RadioData));
    radio.setRetries(5, 15);
    radio.setAutoAck(true);

    radio.openReadingPipe(0, address);
    radio.startListening();
    
    lastRxTime = millis();
}

void Receiver_update() {
    if (radio.available()) {
        RadioData tempData;
        radio.read(&tempData, sizeof(RadioData));
        
        // ==========================================
        // HEADER CHECK
        // ==========================================
        if (tempData.header != 0x55AA) return;

        // ==========================================
        // CHECKSUM
        // ==========================================
        int calc = tempData.throttle + tempData.pitch + tempData.roll + tempData.yaw + tempData.arm;
        if (calc != tempData.checksum) return;

        // Passed all checks! Commit the data.
        rxData = tempData;
        lastRxTime = millis();
        
        // ==========================================
        // MAP (Convert 1000-2000 into degrees for PID)
        // ==========================================
        
        mapped_throttle = constrain(rxData.throttle, 1000, 2000);
        
        // Map 1000-2000 to -30 to +30 degrees for Roll/Pitch
        mapped_roll = map(rxData.roll, 1000, 2000, -3000, 3000) / 100.0f;
        mapped_pitch = map(rxData.pitch, 1000, 2000, -3000, 3000) / 100.0f;
        
        // Map 1000-2000 to -45 to +45 degrees for Yaw
        mapped_yaw = map(rxData.yaw, 1000, 2000, -4500, 4500) / 100.0f;

        // Constrain limits as a safety net
        if (mapped_roll < -30.0f) mapped_roll = -30.0f; 
        if (mapped_roll > 30.0f) mapped_roll = 30.0f;
        if (mapped_pitch < -30.0f) mapped_pitch = -30.0f; 
        if (mapped_pitch > 30.0f) mapped_pitch = 30.0f;
        if (mapped_yaw < -45.0f) mapped_yaw = -45.0f; 
        if (mapped_yaw > 45.0f) mapped_yaw = 45.0f;
    }
}

bool Receiver_hasFailsafe() {
    // Failsafe triggers if no data received for 1 second, or if ARM switch is OFF
    return (millis() - lastRxTime > 1000) || (rxData.arm == 0);
}

int Receiver_getThrottle() { return mapped_throttle; }
float Receiver_getRoll() { return mapped_roll; }
float Receiver_getPitch() { return mapped_pitch; }
float Receiver_getYaw() { return mapped_yaw; }
