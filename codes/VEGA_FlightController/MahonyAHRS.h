#pragma once
#include "FastMath.h"

class MahonyAHRS {
public:
    MahonyAHRS(float kp = 0.5f, float ki = 0.0f);

    void reset();
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);
    
    // Euler angles in degrees
    float getRoll() const { return roll; }
    float getPitch() const { return pitch; }
    float getYaw() const { return yaw; }

    // Raw quaternion access
    Quaternion getQuaternion() const { return q; }

private:
    float twoKp; // 2 * proportional gain (Kp)
    float twoKi; // 2 * integral gain (Ki)
    Quaternion q;
    float integralFBx, integralFBy, integralFBz; // integral error terms scaled by Ki

    float roll;
    float pitch;
    float yaw;

    void computeEuler();
};
