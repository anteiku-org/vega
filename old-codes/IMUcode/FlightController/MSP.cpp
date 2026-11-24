#include "MSP.h"

// MSP Commands required for Betaflight Configurator
#define MSP_API_VERSION 1
#define MSP_FC_VARIANT  2
#define MSP_FC_VERSION  3
#define MSP_BOARD_INFO  4
#define MSP_BUILD_INFO  5
#define MSP_STATUS      101
#define MSP_RAW_IMU     102
#define MSP_ATTITUDE    108

static uint8_t inBuf[256];
static uint8_t mspState = 0;
static uint8_t dataSize = 0;
static uint8_t cmd = 0;
static uint8_t checksum = 0;
static uint8_t offset = 0;

void MSP_init() {
    // Serial is initialized in FlightController.ino
}

void serialize8(uint8_t a) {
    Serial.write(a);
    checksum ^= a;
}

void serialize16(int16_t a) {
    serialize8((a   ) & 0xFF);
    serialize8((a>>8) & 0xFF);
}

void serialize32(uint32_t a) {
    serialize8((a    ) & 0xFF);
    serialize8((a>> 8) & 0xFF);
    serialize8((a>>16) & 0xFF);
    serialize8((a>>24) & 0xFF);
}

void headSerialResponse(uint8_t err, uint8_t s, uint8_t c) {
    Serial.write('$');
    Serial.write('M');
    Serial.write(err ? '!' : '>');
    checksum = 0; // start calculating a new checksum
    serialize8(s);
    serialize8(c);
}

void tailSerialReply() {
    Serial.write(checksum);
}

void evaluateCommand(uint8_t c, AttitudeEstimator& attitude, IMUData& imuData) {
    switch(c) {
        case MSP_API_VERSION:
            headSerialResponse(0, 3, c);
            serialize8(0);  // MSP_PROTOCOL_VERSION
            serialize8(1);  // API_VERSION_MAJOR
            serialize8(41); // API_VERSION_MINOR (1.41 simulates a modern betaflight api)
            tailSerialReply();
            break;
            
        case MSP_FC_VARIANT:
            headSerialResponse(0, 4, c);
            serialize8('B');
            serialize8('T');
            serialize8('F');
            serialize8('L');
            tailSerialReply();
            break;
            
        case MSP_FC_VERSION:
            headSerialResponse(0, 3, c);
            serialize8(4); // MAJOR
            serialize8(3); // MINOR
            serialize8(0); // PATCH
            tailSerialReply();
            break;
            
        case MSP_BOARD_INFO:
            headSerialResponse(0, 4, c);
            serialize8('C');
            serialize8('U');
            serialize8('S');
            serialize8('T'); // CUSTom board
            tailSerialReply();
            break;
            
        case MSP_BUILD_INFO:
            headSerialResponse(0, 11, c);
            for(int i=0; i<11; i++) serialize8('0');
            tailSerialReply();
            break;
            
        case MSP_STATUS:
            headSerialResponse(0, 11, c);
            serialize16(0); // cycleTime
            serialize16(0); // i2c_errors_count
            serialize16(1); // sensor present (1 = ACC)
            serialize32(0); // flight mode flags
            serialize8(0);  // current profile
            tailSerialReply();
            break;
            
        case MSP_RAW_IMU:
            headSerialResponse(0, 18, c);
            // Rough estimation for raw IMU graph
            serialize16((int16_t)(imuData.ax * 512.0f / 9.81f));
            serialize16((int16_t)(imuData.ay * 512.0f / 9.81f));
            serialize16((int16_t)(imuData.az * 512.0f / 9.81f));
            serialize16(imuData.gx);
            serialize16(imuData.gy);
            serialize16(imuData.gz);
            serialize16(0); // magx
            serialize16(0); // magy
            serialize16(0); // magz
            tailSerialReply();
            break;
            
        case MSP_ATTITUDE:
            headSerialResponse(0, 6, c);
            // Betaflight expects roll/pitch in 0.1 deg, yaw in 1 deg
            serialize16((int16_t)(attitude.getRoll() * 10.0f));
            serialize16((int16_t)(attitude.getPitch() * -10.0f)); // Inverted pitch for BF 3D model standard
            serialize16((int16_t)(attitude.getYaw()));
            tailSerialReply();
            break;
            
        default:
            // We don't know this command, send error
            headSerialResponse(1, 0, c);
            tailSerialReply();
            break;
    }
}

void MSP_evaluate(AttitudeEstimator& attitude, IMUData& imuData) {
    while (Serial.available()) {
        uint8_t c = Serial.read();
        
        if (mspState == 0) {
            if (c == '$') mspState = 1;
        } else if (mspState == 1) {
            if (c == 'M') mspState = 2;
            else mspState = 0;
        } else if (mspState == 2) {
            if (c == '<') mspState = 3;
            else mspState = 0;
        } else if (mspState == 3) {
            dataSize = c;
            checksum = c;
            mspState = 4;
        } else if (mspState == 4) {
            cmd = c;
            checksum ^= c;
            offset = 0;
            if (dataSize > 0) mspState = 5;
            else mspState = 6;
        } else if (mspState == 5) {
            inBuf[offset++] = c;
            checksum ^= c;
            if (offset >= dataSize) mspState = 6;
        } else if (mspState == 6) {
            if (checksum == c) {
                // Command is valid, evaluate it!
                evaluateCommand(cmd, attitude, imuData);
            }
            mspState = 0;
        }
    }
}
