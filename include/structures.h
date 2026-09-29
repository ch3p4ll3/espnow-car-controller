#pragma once
#include <stdint.h>

// Structure for received data
struct __attribute__((packed)) CommandMessage {
    uint8_t  leftMotorDirection;    // true = forward, false = backwards
    uint16_t leftMotorSpeed;        // 0 - 100%, 0 = stop

    uint8_t  rightMotorDirection;   // true = forward, false = backwards
    uint16_t rightMotorSpeed;       // 0 - 100%, 0 = stop
};

typedef struct SerialPacket{
    uint8_t header;
    CommandMessage data;
} SerialPacket;

struct __attribute__((packed)) TelemetryMessage {
    uint8_t  leftMotorDirection;    // true = forward, false = backwards
    uint16_t leftMotorSpeed;        // 0 - 100%, 0 = stop
    float    trueLeftSpeed;         // cm/s

    uint8_t  rightMotorDirection;   // true = forward, false = backwards
    uint16_t rightMotorSpeed;       // 0 - 100%, 0 = stop
    float    trueRightSpeed;        // cm/s

    double   lat;
    double   lon;
    double   gpsSpeed;
};
