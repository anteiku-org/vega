#include "MahonyAHRS.h"

MahonyAHRS::MahonyAHRS(float kp, float ki)
    : twoKp(2.0f * kp), twoKi(2.0f * ki),
      q(1.0f, 0.0f, 0.0f, 0.0f),
      integralFBx(0.0f), integralFBy(0.0f), integralFBz(0.0f),
      roll(0.0f), pitch(0.0f), yaw(0.0f) {
}

void MahonyAHRS::reset() {
    q = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    integralFBx = integralFBy = integralFBz = 0.0f;
    roll = pitch = yaw = 0.0f;
}

void MahonyAHRS::updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Convert gyroscope degrees/sec to radians/sec
    gx = degToRad(gx);
    gy = degToRad(gy);
    gz = degToRad(gz);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        recipNorm = fastInvSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity
        halfvx = q.q1 * q.q3 - q.q0 * q.q2;
        halfvy = q.q0 * q.q1 + q.q2 * q.q3;
        halfvz = q.q0 * q.q0 - 0.5f + q.q3 * q.q3;

        // Error is cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dt;
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q.q0;
    qb = q.q1;
    qc = q.q2;
    q.q0 += (-qb * gx - qc * gy - q.q3 * gz);
    q.q1 += (qa * gx + qc * gz - q.q3 * gy);
    q.q2 += (qa * gy - qb * gz + q.q3 * gx);
    q.q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = fastInvSqrt(q.q0 * q.q0 + q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3);
    q.q0 *= recipNorm;
    q.q1 *= recipNorm;
    q.q2 *= recipNorm;
    q.q3 *= recipNorm;

    // Compute Euler angles
    computeEuler();
}

void MahonyAHRS::computeEuler() {
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q.q0 * q.q1 + q.q2 * q.q3);
    float cosr_cosp = 1.0f - 2.0f * (q.q1 * q.q1 + q.q2 * q.q2);
    roll = radToDeg(atan2f(sinr_cosp, cosr_cosp));

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q.q0 * q.q2 - q.q3 * q.q1);
    if (fabsf(sinp) >= 1.0f)
        pitch = radToDeg(copysignf(1.57079632679f, sinp)); // use 90 degrees if out of range
    else
        pitch = radToDeg(asinf(sinp));

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q.q0 * q.q3 + q.q1 * q.q2);
    float cosy_cosp = 1.0f - 2.0f * (q.q2 * q.q2 + q.q3 * q.q3);
    yaw = radToDeg(atan2f(siny_cosp, cosy_cosp));
}
