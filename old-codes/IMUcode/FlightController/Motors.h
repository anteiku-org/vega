#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

// ESP32 Pins connected to the ESC signal wires
#define MOTOR_FR_PIN 13 // Front Right
#define MOTOR_FL_PIN 12 // Front Left
#define MOTOR_RL_PIN 14 // Rear Left
#define MOTOR_RR_PIN 27 // Rear Right

// PWM Configuration for SimonK ESCs
#define ESC_PWM_FREQ 250 // 250Hz is a good fast update rate for SimonK
#define ESC_PWM_RES  12  // 12-bit resolution (0-4095)

// Define ESC pulse widths in microseconds
#define ESC_MIN_PULSE 1000 // Minimum throttle (Motors stopped)
#define ESC_MAX_PULSE 2000 // Maximum throttle (Full speed)

void Motors_init();
void Motors_write(int throttle, int rollPID, int pitchPID, int yawPID);
void Motors_stopAll();

#endif
