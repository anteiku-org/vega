#include "AttitudeEstimator.h"
#include <math.h>

AttitudeEstimator::AttitudeEstimator()
{
    // Identity quaternion
    qw = 1.0f;
    qx = 0.0f;
    qy = 0.0f;
    qz = 0.0f;

    roll = 0.0f;
    pitch = 0.0f;
    yaw = 0.0f;
}

void AttitudeEstimator::initialize(const IMUData &data)
{
    // For Step 8F-1:
    // initialize using accelerometer roll and pitch
    // while leaving yaw at zero.

    roll = calculateAccelRoll(data);
    pitch = calculateAccelPitch(data);
    yaw = 0.0f;

    // Convert initial Euler angles to quaternion

    float phi = roll * PI / 180.0f;
    float theta = pitch * PI / 180.0f;
    float psi = yaw * PI / 180.0f;

    float cr = cos(phi * 0.5f);
    float sr = sin(phi * 0.5f);

    float cp = cos(theta * 0.5f);
    float sp = sin(theta * 0.5f);

    float cy = cos(psi * 0.5f);
    float sy = sin(psi * 0.5f);

    qw = cr * cp * cy + sr * sp * sy;
    qx = sr * cp * cy - cr * sp * sy;
    qy = cr * sp * cy + sr * cp * sy;
    qz = cr * cp * sy - sr * sp * cy;

    normalizeQuaternion();
}

void AttitudeEstimator::update(const IMUData &data, float dt)
{
    if (dt <= 0.0f || dt > 0.1f)
        return;

    // Convert gyro rates from deg/s to rad/s

    float gx = data.gx * PI / 180.0f;
    float gy = data.gy * PI / 180.0f;
    float gz = data.gz * PI / 180.0f;

    // Quaternion derivative
    //
    // q_dot = 0.5 * q ⊗ omega

    float qDotW =
        0.5f * (
            -qx * gx
            -qy * gy
            -qz * gz
        );

    float qDotX =
        0.5f * (
             qw * gx
            +qy * gz
            -qz * gy
        );

    float qDotY =
        0.5f * (
             qw * gy
            -qx * gz
            +qz * gx
        );

    float qDotZ =
        0.5f * (
             qw * gz
            +qx * gy
            -qy * gx
        );

    // Integrate

    qw += qDotW * dt;
    qx += qDotX * dt;
    qy += qDotY * dt;
    qz += qDotZ * dt;

    // Keep quaternion normalized

    normalizeQuaternion();

    // Convert quaternion to Euler angles (gyro-only estimate)
    updateEulerAngles();

    // ====================================================
    // COMPLEMENTARY FILTER
    // Blend accelerometer to prevent gyro drift!
    // alpha = 0.98 means 98% trust gyro, 2% trust accel
    // ====================================================
    const float alpha = 0.98f;

    float accelRoll = calculateAccelRoll(data);
    float accelPitch = calculateAccelPitch(data);

    // Only correct when acceleration is close to 1g (drone is not accelerating hard)
    float accelMag = sqrt(data.ax * data.ax + data.ay * data.ay + data.az * data.az);
    float gravityMag = 9.80665f;

    if (accelMag > gravityMag * 0.8f && accelMag < gravityMag * 1.2f) {
        roll  = alpha * roll  + (1.0f - alpha) * accelRoll;
        pitch = alpha * pitch + (1.0f - alpha) * accelPitch;
    }
    // Yaw: no magnetometer, so yaw stays gyro-only (will drift slowly, that's expected)

    // Rebuild quaternion from the corrected Euler angles
    float phi   = roll  * PI / 180.0f;
    float theta = pitch * PI / 180.0f;
    float psi   = yaw   * PI / 180.0f;

    float cr = cos(phi * 0.5f);
    float sr = sin(phi * 0.5f);
    float cp = cos(theta * 0.5f);
    float sp = sin(theta * 0.5f);
    float cy = cos(psi * 0.5f);
    float sy = sin(psi * 0.5f);

    qw = cr * cp * cy + sr * sp * sy;
    qx = sr * cp * cy - cr * sp * sy;
    qy = cr * sp * cy + sr * cp * sy;
    qz = cr * cp * sy - sr * sp * cy;

    normalizeQuaternion();
}

void AttitudeEstimator::normalizeQuaternion()
{
    float magnitude =
        sqrt(
            qw * qw
            + qx * qx
            + qy * qy
            + qz * qz
        );

    if (magnitude <= 0.0f)
    {
        // Safety fallback
        qw = 1.0f;
        qx = 0.0f;
        qy = 0.0f;
        qz = 0.0f;

        return;
    }

    qw /= magnitude;
    qx /= magnitude;
    qy /= magnitude;
    qz /= magnitude;
}

void AttitudeEstimator::updateEulerAngles()
{
    // Roll

    float sinRoll =
        2.0f * (qw * qx + qy * qz);

    float cosRoll =
        1.0f
        - 2.0f * (qx * qx + qy * qy);

    roll =
        atan2(sinRoll, cosRoll)
        * 180.0f / PI;

    // Pitch

    float sinPitch =
        2.0f * (qw * qy - qz * qx);

    // Protect against numerical error
    // slightly outside [-1, +1]

    if (sinPitch > 1.0f)
        sinPitch = 1.0f;

    if (sinPitch < -1.0f)
        sinPitch = -1.0f;

    pitch =
        asin(sinPitch)
        * 180.0f / PI;

    // Yaw

    float sinYaw =
        2.0f * (qw * qz + qx * qy);

    float cosYaw =
        1.0f
        - 2.0f * (qy * qy + qz * qz);

    yaw =
        atan2(sinYaw, cosYaw)
        * 180.0f / PI;
}

float AttitudeEstimator::calculateAccelRoll(const IMUData &data)
{
    return atan2(
        data.ay,
        data.az
    ) * 180.0f / PI;
}

float AttitudeEstimator::calculateAccelPitch(const IMUData &data)
{
    return atan2(
        -data.ax,
        sqrt(
            data.ay * data.ay
            + data.az * data.az
        )
    ) * 180.0f / PI;
}

float AttitudeEstimator::getRoll()
{
    return roll;
}

float AttitudeEstimator::getPitch()
{
    return pitch;
}

float AttitudeEstimator::getYaw()
{
    return yaw;
}