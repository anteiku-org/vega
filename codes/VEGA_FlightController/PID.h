#pragma once
#include "Config.h"
#include "FastMath.h"

// Single-axis PID controller with low-pass filtered D-term on measurement
class AxisPID {
public:
    AxisPID();
    void setGains(float kp, float ki, float kd);
    void reset();
    float update(float targetRate, float currentRate, float dt);

private:
    float kp, ki, kd;
    float integral;
    float prevRate;
    float dTermFiltered;
    float lpfAlpha; // D-term low-pass filter coefficient
};

// Cascaded Angle + Rate controller for Roll and Pitch
class CascadedController {
public:
    CascadedController();

    void setAngleGains(float kp);
    void setRateGains(float kp, float ki, float kd);
    void reset();

    // Returns motor mixer correction in microseconds
    float update(float targetAngle, float currentAngle, float currentRate, float dt, bool isAngleMode);

private:
    float angleKp;
    AxisPID ratePid;
};

// Complete 3-Axis Multirotor Flight Controller
class FlightPID {
public:
    FlightPID();
    void reset();

    void compute(float targetRollAngle, float targetPitchAngle, float targetYawRate,
                 float currentRollAngle, float currentPitchAngle,
                 float gyroRateX, float gyroRateY, float gyroRateZ,
                 float dt, bool isAngleMode,
                 float& outRoll, float& outPitch, float& outYaw);

private:
    CascadedController rollController;
    CascadedController pitchController;
    AxisPID yawRatePid;
};
