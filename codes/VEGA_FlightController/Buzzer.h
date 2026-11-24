#pragma once
#include <Arduino.h>
#include "Config.h"

enum BuzzerPattern {
    BUZZER_SILENT = 0,
    BUZZER_BOOT_SUCCESS,
    BUZZER_ARMED,
    BUZZER_DISARMED,
    BUZZER_BATTERY_LOW,
    BUZZER_FAILSAFE_ALARM
};

class FlightBuzzer {
public:
    FlightBuzzer();

    void begin();
    void play(BuzzerPattern pattern);
    void update();

private:
    BuzzerPattern currentPattern;
    unsigned long stateStartTime;
    uint8_t stepIndex;

    void setBuzzer(bool on);
};
