#ifndef RECEIVER_H
#define RECEIVER_H

#include <Arduino.h>

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

void Receiver_init();
void Receiver_update();

// Failsafe: Returns true if connection to the app is lost
bool Receiver_hasFailsafe();

// Getters for the pilot's joystick commands
int Receiver_getThrottle(); // Output: 1000 (motors off) to 2000 (full speed)
float Receiver_getRoll();   // Output: -30.0 to 30.0 degrees (target angle)
float Receiver_getPitch();  // Output: -30.0 to 30.0 degrees (target angle)
float Receiver_getYaw();    // Output: -45.0 to 45.0 degrees per second (rotation rate)

#endif
