#include <SPI.h>
#include <RF24.h>

// NRF24L01 CE, CSN pins for Arduino Uno/Nano
RF24 radio(9, 10);

const byte address[6] = "ABCDE";

// Highly robust data structure with Header and Checksum
#pragma pack(push, 1)
struct RadioData {
  uint16_t header;
  
  uint16_t throttle;
  uint16_t pitch;
  uint16_t roll;
  uint16_t yaw;
  
  uint8_t arm;
  
  uint16_t checksum;
};
#pragma pack(pop)

RadioData data;

// Analog pins for the Joysticks
// Ensure these match your actual wiring!
const int throttlePin = A0; // Left Stick Y
const int yawPin = A1;      // Left Stick X
const int pitchPin = A2;    // Right Stick Y
const int rollPin = A3;     // Right Stick X

const int armSwitch = 2;
const int armLED = 4;

void setup() {
  pinMode(armSwitch, INPUT_PULLUP);
  pinMode(armLED, OUTPUT);
  digitalWrite(armLED, LOW);

  Serial.begin(115200);

  if (radio.begin()) {
    Serial.println("NRF24L01 Initialized Successfully");
  } else {
    Serial.println("NRF24L01 Initialization FAILED");
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(108); // Avoid WiFi interference
  radio.setPayloadSize(sizeof(RadioData));
  radio.setRetries(5, 15);
  radio.setAutoAck(true);
  
  radio.openWritingPipe(address);
  radio.stopListening();
}

void loop() {
  // Read joysticks
  int rawThrottle = analogRead(throttlePin);
  int rawYaw      = analogRead(yawPin);
  int rawPitch    = analogRead(pitchPin);
  int rawRoll     = analogRead(rollPin);

  // Set the header
  data.header = 0x55AA;

  // Map to Flight Controller ranges
  // Throttle: 1000 to 2000
  data.throttle = map(rawThrottle, 0, 1023, 1000, 2000);
  
  // Yaw, Pitch, Roll: 1000 to 2000
  // (We will let the ESP32 receiver convert 1000-2000 back into -45/+45 angles for safety)
  data.yaw = map(rawYaw, 0, 1023, 1000, 2000);
  data.pitch = map(rawPitch, 0, 1023, 1000, 2000);
  data.roll = map(rawRoll, 0, 1023, 1000, 2000);

  // Read ARM switch (LOW means switch connected to GND)
  data.arm = (digitalRead(armSwitch) == LOW) ? 1 : 0;
  digitalWrite(armLED, data.arm);

  // Calculate Checksum
  data.checksum = data.throttle + data.pitch + data.roll + data.yaw + data.arm;

  // Send the data packet
  bool success = radio.write(&data, sizeof(data));

  if (success) {
    Serial.print("SEND OK | ");
  } else {
    Serial.print("SEND FAIL | ");
  }

  Serial.print("Thr: "); Serial.print(data.throttle);
  Serial.print(" Y: "); Serial.print(data.yaw);
  Serial.print(" P: "); Serial.print(data.pitch);
  Serial.print(" R: "); Serial.print(data.roll);
  Serial.print(" ARM: "); Serial.println(data.arm ? "ON" : "OFF");

  delay(20); // 50Hz update rate
}
