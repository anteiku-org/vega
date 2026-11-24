#pragma once
#include <Arduino.h>
#include <math.h>

struct Vector3f {
    float x;
    float y;
    float z;

    Vector3f() : x(0.0f), y(0.0f), z(0.0f) {}
    Vector3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

struct Quaternion {
    float q0, q1, q2, q3;

    Quaternion() : q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f) {}
    Quaternion(float w, float x, float y, float z) : q0(w), q1(x), q2(y), q3(z) {}
};

// Fast inverse square root (Quake III algorithm for real-time attitude estimation)
inline float fastInvSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

inline float degToRad(float deg) {
    return deg * 0.017453292519943295f;
}

inline float radToDeg(float rad) {
    return rad * 57.29577951308232f;
}

inline float clampFloat(float val, float minVal, float maxVal) {
    if (val < minVal) return minVal;
    if (val > maxVal) return maxVal;
    return val;
}
