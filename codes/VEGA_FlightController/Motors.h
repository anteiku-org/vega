#pragma once
#include "Config.h"
#include "FastMath.h"

class MotorDriver {
public:
    MotorDriver();

    void begin();
    void setArmed(bool armed);
    bool isArmed() const { return armedState; }

    // Mix and send pulses (Quad-X layout)
    void write(float throttle, float rollPID, float pitchPID, float yawPID);
    void stopAll();

    // Pulse readouts for telemetry
    int getMotor1Pulse() const { return m1; }
    int getMotor2Pulse() const { return m2; }
    int getMotor3Pulse() const { return m3; }
    int getMotor4Pulse() const { return m4; }

private:
    bool armedState;
    int m1, m2, m3, m4;

    void writePulseToPin(uint8_t pin, uint8_t channel, int pulseUs);
    uint32_t usToDuty(int us);
};
