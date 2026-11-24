#ifndef PID_H
#define PID_H

#include <Arduino.h>

void PID_init();
void PID_reset();

// Computes the PID correction values for motors
// rollAngle, pitchAngle are in degrees.
// yawRate is the gyroscope Z axis rotation rate in degrees per second.
// dt is the time passed since the last loop in seconds.
void PID_compute(float rollAngle, float pitchAngle, float yawRate, 
                 float rollSetpoint, float pitchSetpoint, float yawSetpoint, 
                 float dt, 
                 int& outRoll, int& outPitch, int& outYaw);

// Exposed for PID Tuning Web Server
extern float kp_roll, ki_roll, kd_roll;
extern float kp_pitch, ki_pitch, kd_pitch;
extern float kp_yaw, ki_yaw, kd_yaw;

#endif
