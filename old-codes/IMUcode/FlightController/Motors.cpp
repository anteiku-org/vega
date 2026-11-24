#include "Motors.h"

// Helper function to convert microseconds to duty cycle (0-4095)
// For 250Hz, Period = 4000us. 
// Duty = (Pulse_us / 4000) * 4096 = Pulse_us * 1.024
int usToDuty(int us) {
    if (us < ESC_MIN_PULSE) us = ESC_MIN_PULSE;
    if (us > ESC_MAX_PULSE) us = ESC_MAX_PULSE;
    return (int)(us * 1.024f); 
}

void Motors_init() {
    // Using ESP32 Core 3.0+ ledc API
    ledcAttach(MOTOR_FR_PIN, ESC_PWM_FREQ, ESC_PWM_RES);
    ledcAttach(MOTOR_FL_PIN, ESC_PWM_FREQ, ESC_PWM_RES);
    ledcAttach(MOTOR_RL_PIN, ESC_PWM_FREQ, ESC_PWM_RES);
    ledcAttach(MOTOR_RR_PIN, ESC_PWM_FREQ, ESC_PWM_RES);

    // Ensure motors start totally off!
    Motors_stopAll();
}

void Motors_write(int throttle, int rollPID, int pitchPID, int yawPID) {
    // If throttle is all the way down, force motors off for safety!
    if (throttle < 1050) {
        Motors_stopAll();
        return;
    }

    // Quadcopter X Configuration Motor Mixing
    // Front Right (Counter-Clockwise)
    int fr = throttle - rollPID - pitchPID + yawPID;
    
    // Front Left (Clockwise)
    int fl = throttle + rollPID - pitchPID - yawPID;
    
    // Rear Left (Counter-Clockwise)
    int rl = throttle + rollPID + pitchPID + yawPID;
    
    // Rear Right (Clockwise)
    int rr = throttle - rollPID + pitchPID - yawPID;

    // CRITICAL: Clamp each motor to safe ESC range after mixing!
    fr = constrain(fr, ESC_MIN_PULSE, ESC_MAX_PULSE);
    fl = constrain(fl, ESC_MIN_PULSE, ESC_MAX_PULSE);
    rl = constrain(rl, ESC_MIN_PULSE, ESC_MAX_PULSE);
    rr = constrain(rr, ESC_MIN_PULSE, ESC_MAX_PULSE);

    // Send PWM pulses to ESCs
    ledcWrite(MOTOR_FR_PIN, usToDuty(fr));
    ledcWrite(MOTOR_FL_PIN, usToDuty(fl));
    ledcWrite(MOTOR_RL_PIN, usToDuty(rl));
    ledcWrite(MOTOR_RR_PIN, usToDuty(rr));
}

void Motors_stopAll() {
    int stopDuty = usToDuty(ESC_MIN_PULSE);
    ledcWrite(MOTOR_FR_PIN, stopDuty);
    ledcWrite(MOTOR_FL_PIN, stopDuty);
    ledcWrite(MOTOR_RL_PIN, stopDuty);
    ledcWrite(MOTOR_RR_PIN, stopDuty);
}
