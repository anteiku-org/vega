#include "Buzzer.h"

FlightBuzzer::FlightBuzzer()
    : currentPattern(BUZZER_SILENT), stateStartTime(0), stepIndex(0) {
}

void FlightBuzzer::begin() {
    pinMode(PIN_BUZZER, OUTPUT);
    setBuzzer(false);
}

void FlightBuzzer::setBuzzer(bool on) {
    digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
}

void FlightBuzzer::play(BuzzerPattern pattern) {
    currentPattern = pattern;
    stepIndex = 0;
    stateStartTime = millis();
}

void FlightBuzzer::update() {
    if (currentPattern == BUZZER_SILENT) {
        setBuzzer(false);
        return;
    }

    unsigned long elapsed = millis() - stateStartTime;

    switch (currentPattern) {
        case BUZZER_BOOT_SUCCESS:
            // Beep (80ms), Pause (80ms), Beep (80ms), Pause (80ms), Beep (150ms)
            if (elapsed < 80) setBuzzer(true);
            else if (elapsed < 160) setBuzzer(false);
            else if (elapsed < 240) setBuzzer(true);
            else if (elapsed < 320) setBuzzer(false);
            else if (elapsed < 470) setBuzzer(true);
            else {
                setBuzzer(false);
                currentPattern = BUZZER_SILENT;
            }
            break;

        case BUZZER_ARMED:
            // Single confident 250ms beep
            if (elapsed < 250) setBuzzer(true);
            else {
                setBuzzer(false);
                currentPattern = BUZZER_SILENT;
            }
            break;

        case BUZZER_DISARMED:
            // 2 gentle 70ms beeps
            if (elapsed < 70) setBuzzer(true);
            else if (elapsed < 140) setBuzzer(false);
            else if (elapsed < 210) setBuzzer(true);
            else {
                setBuzzer(false);
                currentPattern = BUZZER_SILENT;
            }
            break;

        case BUZZER_BATTERY_LOW:
            // Periodic warning double-beep every 1.5 seconds
            if (elapsed < 100) setBuzzer(true);
            else if (elapsed < 200) setBuzzer(false);
            else if (elapsed < 300) setBuzzer(true);
            else if (elapsed < 1500) setBuzzer(false);
            else {
                stateStartTime = millis(); // Loop pattern
            }
            break;

        case BUZZER_FAILSAFE_ALARM:
            // Fast continuous alarm beep (100ms ON / 100ms OFF)
            if ((elapsed / 100) % 2 == 0) setBuzzer(true);
            else setBuzzer(false);
            break;

        default:
            setBuzzer(false);
            currentPattern = BUZZER_SILENT;
            break;
    }
}
