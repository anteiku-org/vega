#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include "MPU6500.h"

class AttitudeEstimator {
public:

  AttitudeEstimator();

  void initialize(const IMUData &data);

  void update(const IMUData &data, float dt);

  float getRoll();
  float getPitch();
  float getYaw();

private:

  // Quaternion representing attitude
  float qw;
  float qx;
  float qy;
  float qz;

  // Euler angles derived from quaternion
  float roll;
  float pitch;
  float yaw;

  void normalizeQuaternion();

  float calculateAccelRoll(const IMUData &data);
  float calculateAccelPitch(const IMUData &data);

  void updateEulerAngles();
};

#endif