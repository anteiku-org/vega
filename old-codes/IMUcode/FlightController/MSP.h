#ifndef MSP_H
#define MSP_H

#include <Arduino.h>
#include "AttitudeEstimator.h"

// Initialize the MSP handler
void MSP_init();

// Evaluate and parse incoming serial data for MSP commands
void MSP_evaluate(AttitudeEstimator& attitude, IMUData& imuData);

#endif
