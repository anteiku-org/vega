#include "Motors.h"

// 14-bit resolution at 250Hz: Max count = 16384
// Period = 4000us. Counts per us = 16384 / 4000 = 4.096
static const uint32_t LEDC_RES_BITS = 14;
static const uint32_t MAX_DUTY = (1 << LEDC_RES_BITS) - 1;

#define CH_M1 0
#define CH_M2 1
#define CH_M3 2
#define CH_M4 3

MotorDriver::MotorDriver()
    : armedState(false), m1(1000), m2(1000), m3(1000), m4(1000) {
}

uint32_t MotorDriver::usToDuty(int us) {
    us = (int)clampFloat((float)us, (float)ESC_MIN_PULSE_US, (float)ESC_MAX_PULSE_US);
    return (uint32_t)(us * 4.096f);
}

void MotorDriver::begin() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttach(PIN_MOTOR_RR, ESC_PWM_FREQ_HZ, LEDC_RES_BITS);
    ledcAttach(PIN_MOTOR_FR, ESC_PWM_FREQ_HZ, LEDC_RES_BITS);
    ledcAttach(PIN_MOTOR_RL, ESC_PWM_FREQ_HZ, LEDC_RES_BITS);
    ledcAttach(PIN_MOTOR_FL, ESC_PWM_FREQ_HZ, LEDC_RES_BITS);
#else
    ledcSetup(CH_M1, ESC_PWM_FREQ_HZ, LEDC_RES_BITS);
    ledcAttachPin(PIN_MOTOR_RR, CH_M1);

    ledcSetup(CH_M2, ESC_PWM_FREQ_HZ, LEDC_RES_BITS);
    ledcAttachPin(PIN_MOTOR_FR, CH_M2);

    ledcSetup(CH_M3, ESC_PWM_FREQ_HZ, LEDC_RES_BITS);
    ledcAttachPin(PIN_MOTOR_RL, CH_M3);

    ledcSetup(CH_M4, ESC_PWM_FREQ_HZ, LEDC_RES_BITS);
    ledcAttachPin(PIN_MOTOR_FL, CH_M4);
#endif

    stopAll();
}

void MotorDriver::setArmed(bool armed) {
    armedState = armed;
    if (!armedState) {
        stopAll();
    }
}

void MotorDriver::writePulseToPin(uint8_t pin, uint8_t channel, int pulseUs) {
    uint32_t duty = usToDuty(pulseUs);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(pin, duty);
#else
    ledcWrite(channel, duty);
#endif
}

void MotorDriver::stopAll() {
    m1 = m2 = m3 = m4 = ESC_STOP_PULSE_US;
    writePulseToPin(PIN_MOTOR_RR, CH_M1, ESC_STOP_PULSE_US);
    writePulseToPin(PIN_MOTOR_FR, CH_M2, ESC_STOP_PULSE_US);
    writePulseToPin(PIN_MOTOR_RL, CH_M3, ESC_STOP_PULSE_US);
    writePulseToPin(PIN_MOTOR_FL, CH_M4, ESC_STOP_PULSE_US);
}

void MotorDriver::write(float throttle, float rollPID, float pitchPID, float yawPID) {
    if (!armedState || throttle < 1050.0f) {
        if (armedState && throttle >= 1000.0f) {
            // Armed idle spool-up (AirMode idle)
            m1 = m2 = m3 = m4 = ESC_ARM_PULSE_US;
            writePulseToPin(PIN_MOTOR_RR, CH_M1, m1);
            writePulseToPin(PIN_MOTOR_FR, CH_M2, m2);
            writePulseToPin(PIN_MOTOR_RL, CH_M3, m3);
            writePulseToPin(PIN_MOTOR_FL, CH_M4, m4);
        } else {
            stopAll();
        }
        return;
    }

    // Standard Betaflight Quad-X Mixer:
    // Motor 1 (Rear-Right, CCW): Throttle - Roll + Pitch + Yaw
    // Motor 2 (Front-Right, CW): Throttle - Roll - Pitch - Yaw
    // Motor 3 (Rear-Left, CW):   Throttle + Roll + Pitch - Yaw
    // Motor 4 (Front-Left, CCW): Throttle + Roll - Pitch + Yaw
    float pulse1 = throttle - rollPID + pitchPID + yawPID;
    float pulse2 = throttle - rollPID - pitchPID - yawPID;
    float pulse3 = throttle + rollPID + pitchPID - yawPID;
    float pulse4 = throttle + rollPID - pitchPID + yawPID;

    m1 = (int)clampFloat(pulse1, (float)ESC_MIN_PULSE_US, (float)ESC_MAX_PULSE_US);
    m2 = (int)clampFloat(pulse2, (float)ESC_MIN_PULSE_US, (float)ESC_MAX_PULSE_US);
    m3 = (int)clampFloat(pulse3, (float)ESC_MIN_PULSE_US, (float)ESC_MAX_PULSE_US);
    m4 = (int)clampFloat(pulse4, (float)ESC_MIN_PULSE_US, (float)ESC_MAX_PULSE_US);

    writePulseToPin(PIN_MOTOR_RR, CH_M1, m1);
    writePulseToPin(PIN_MOTOR_FR, CH_M2, m2);
    writePulseToPin(PIN_MOTOR_RL, CH_M3, m3);
    writePulseToPin(PIN_MOTOR_FL, CH_M4, m4);
}
