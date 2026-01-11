#include "MachinePowerSync.h"

namespace {
constexpr uint32_t CHECK_INTERVAL_MS = 5000;
constexpr uint32_t PRESS_MS = 2000;
} // namespace

MachinePowerSync::MachinePowerSync(uint8_t powerLedSensePin, uint8_t powerButtonPin)
    : powerLedSensePin(powerLedSensePin), powerButtonPin(powerButtonPin) {}

void MachinePowerSync::setup() {
    pinMode(powerLedSensePin, INPUT_PULLDOWN);

    pinMode(powerButtonPin, OUTPUT);
    digitalWrite(powerButtonPin, LOW);

    nextCheckMs = 0;
}

void MachinePowerSync::setDesiredOn(const bool desiredOn) { this->desiredOn = desiredOn; }

bool MachinePowerSync::readPowerLed() const { return digitalRead(powerLedSensePin) == HIGH; }

bool MachinePowerSync::timeReached(const uint32_t nowMs, const uint32_t targetMs) {
    return static_cast<int32_t>(nowMs - targetMs) >= 0;
}

void MachinePowerSync::startMomentaryPress(const uint32_t nowMs) {
    pressActive = true;
    pressStartMs = nowMs;
    digitalWrite(powerButtonPin, HIGH);
    ESP_LOGI(LOG_TAG, "Momentary press start (desiredOn=%d, powerLed=%d)", desiredOn ? 1 : 0, readPowerLed() ? 1 : 0);
}

void MachinePowerSync::stopMomentaryPress() {
    digitalWrite(powerButtonPin, LOW);
    pressActive = false;
}

void MachinePowerSync::loop() {
    const uint32_t nowMs = millis();

    if (pressActive) {
        if (timeReached(nowMs, pressStartMs + PRESS_MS)) {
            stopMomentaryPress();
            nextCheckMs = nowMs + CHECK_INTERVAL_MS;
            ESP_LOGI(LOG_TAG, "Momentary press end");
        }
        return;
    }

    if (!timeReached(nowMs, nextCheckMs)) {
        return;
    }

    nextCheckMs = nowMs + CHECK_INTERVAL_MS;

    const bool powerLedOn = readPowerLed();
    if (desiredOn) {
        // GM ON (heater setpoint > 0): if LED has power -> no action, else press button to turn ON
        if (!powerLedOn) {
            startMomentaryPress(nowMs);
        }
    } else {
        // GM OFF/STANDBY (heater setpoint == 0): if LED has power -> press button to turn OFF, else no action
        if (powerLedOn) {
            startMomentaryPress(nowMs);
        }
    }
}
