#include "PID.h"

// -----------------------------------------------------------------------------
// AxisPID (Inner Rate Loop)
// -----------------------------------------------------------------------------
AxisPID::AxisPID()
    : kp(0.0f), ki(0.0f), kd(0.0f),
      integral(0.0f), prevRate(0.0f), dTermFiltered(0.0f), lpfAlpha(0.25f) {
}

void AxisPID::setGains(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
}

void AxisPID::reset() {
    integral = 0.0f;
    prevRate = 0.0f;
    dTermFiltered = 0.0f;
}

float AxisPID::update(float targetRate, float currentRate, float dt) {
    if (dt <= 0.0f) return 0.0f;

    float rateError = targetRate - currentRate;

    // 1. Proportional Term
    float pTerm = kp * rateError;

    // 2. Integral Term with Anti-Windup Clamping
    integral += ki * rateError * dt;
    integral = clampFloat(integral, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);

    // 3. Derivative Term on Measurement (avoid derivative kick from stick step changes)
    float deltaRate = (currentRate - prevRate) / dt;
    prevRate = currentRate;
    float rawDTerm = -kd * deltaRate;

    // First-order Low-Pass Filter on D-term to strip high-frequency motor vibration
    dTermFiltered = dTermFiltered + lpfAlpha * (rawDTerm - dTermFiltered);

    // Sum all terms
    return pTerm + integral + dTermFiltered;
}

// -----------------------------------------------------------------------------
// CascadedController (Outer Angle -> Inner Rate)
// -----------------------------------------------------------------------------
CascadedController::CascadedController() : angleKp(ANGLE_KP_ROLL) {}

void CascadedController::setAngleGains(float p) {
    angleKp = p;
}

void CascadedController::setRateGains(float p, float i, float d) {
    ratePid.setGains(p, i, d);
}

void CascadedController::reset() {
    ratePid.reset();
}

float CascadedController::update(float targetAngle, float currentAngle, float currentRate, float dt, bool isAngleMode) {
    float targetRate = 0.0f;

    if (isAngleMode) {
        // Outer Loop: Angle Error -> Desired Angular Velocity (deg/s)
        float angleError = targetAngle - currentAngle;
        targetRate = angleKp * angleError;
        targetRate = clampFloat(targetRate, -MAX_RATE_DPS, MAX_RATE_DPS);
    } else {
        // Acro / Rate Mode: Direct stick-to-rate mapping
        targetRate = targetAngle; // In Acro mode, stick input represents rate directly
    }

    // Inner Loop: Rate Error -> ESC Output Delta
    return ratePid.update(targetRate, currentRate, dt);
}

// -----------------------------------------------------------------------------
// FlightPID Manager
// -----------------------------------------------------------------------------
FlightPID::FlightPID() {
    rollController.setAngleGains(ANGLE_KP_ROLL);
    rollController.setRateGains(RATE_KP_ROLL, RATE_KI_ROLL, RATE_KD_ROLL);

    pitchController.setAngleGains(ANGLE_KP_PITCH);
    pitchController.setRateGains(RATE_KP_PITCH, RATE_KI_PITCH, RATE_KD_PITCH);

    yawRatePid.setGains(RATE_KP_YAW, RATE_KI_YAW, RATE_KD_YAW);
}

void FlightPID::reset() {
    rollController.reset();
    pitchController.reset();
    yawRatePid.reset();
}

void FlightPID::compute(float targetRollAngle, float targetPitchAngle, float targetYawRate,
                        float currentRollAngle, float currentPitchAngle,
                        float gyroRateX, float gyroRateY, float gyroRateZ,
                        float dt, bool isAngleMode,
                        float& outRoll, float& outPitch, float& outYaw) {
    outRoll = rollController.update(targetRollAngle, currentRollAngle, gyroRateX, dt, isAngleMode);
    outPitch = pitchController.update(targetPitchAngle, currentPitchAngle, gyroRateY, dt, isAngleMode);
    outYaw = yawRatePid.update(targetYawRate, gyroRateZ, dt);
}
