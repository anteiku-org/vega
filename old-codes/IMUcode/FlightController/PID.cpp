#include "PID.h"

// Basic PID tuning values - These WILL need to be adjusted during testing!
// P = Proportional (immediate reaction to tilt)
// I = Integral (long term correction to keep drone perfectly flat against wind)
// D = Derivative (predicts movement and dampens wobbles)
float kp_roll = 0.7f, ki_roll = 0.0f, kd_roll = 0.2f;
float kp_pitch = 0.7f, ki_pitch = 0.0f, kd_pitch = 0.0f;
float kp_yaw = 2.0f, ki_yaw = 0.0f, kd_yaw = 0.0f;

float i_roll = 0, i_pitch = 0, i_yaw = 0;
float prev_error_roll = 0, prev_error_pitch = 0, prev_error_yaw = 0;

void PID_init() {
    PID_reset();
}

void PID_reset() {
    i_roll = 0; i_pitch = 0; i_yaw = 0;
    prev_error_roll = 0; prev_error_pitch = 0; prev_error_yaw = 0;
}

void PID_compute(float rollAngle, float pitchAngle, float yawRate, 
                 float rollSetpoint, float pitchSetpoint, float yawSetpoint, 
                 float dt, 
                 int& outRoll, int& outPitch, int& outYaw) 
{
    // Prevent division by zero
    if (dt <= 0.00001f) dt = 0.00001f;

    // 1. Calculate errors (Difference between what the pilot wants and what the drone is doing)
    float error_roll = rollSetpoint - rollAngle;
    float error_pitch = pitchSetpoint - pitchAngle;
    float error_yaw = yawSetpoint - yawRate; // Yaw uses rotation rate instead of absolute angle

    // 2. Proportional Term
    float p_roll = kp_roll * error_roll;
    float p_pitch = kp_pitch * error_pitch;
    float p_yaw = kp_yaw * error_yaw;

    // 3. Integral Term
    i_roll += ki_roll * error_roll * dt;
    i_pitch += ki_pitch * error_pitch * dt;
    i_yaw += ki_yaw * error_yaw * dt;

    // Anti-windup (prevents the integral from growing infinitely large when motors are saturated)
    if(i_roll > 400) i_roll = 400; if(i_roll < -400) i_roll = -400;
    if(i_pitch > 400) i_pitch = 400; if(i_pitch < -400) i_pitch = -400;
    if(i_yaw > 400) i_yaw = 400; if(i_yaw < -400) i_yaw = -400;

    // 4. Derivative Term
    float d_roll = kd_roll * ((error_roll - prev_error_roll) / dt);
    float d_pitch = kd_pitch * ((error_pitch - prev_error_pitch) / dt);
    float d_yaw = kd_yaw * ((error_yaw - prev_error_yaw) / dt);

    prev_error_roll = error_roll;
    prev_error_pitch = error_pitch;
    prev_error_yaw = error_yaw;

    // 5. Total Output
    outRoll = (int)(p_roll + i_roll + d_roll);
    outPitch = (int)(p_pitch + i_pitch + d_pitch);
    outYaw = (int)(p_yaw + i_yaw + d_yaw);

    // Clamp PID output to prevent violent motor corrections
    const int PID_MAX = 400;
    outRoll  = constrain(outRoll,  -PID_MAX, PID_MAX);
    outPitch = constrain(outPitch, -PID_MAX, PID_MAX);
    outYaw   = constrain(outYaw,   -PID_MAX, PID_MAX);
}
