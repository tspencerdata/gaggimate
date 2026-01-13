#include "MachinePowerSync.h"

namespace {
constexpr uint32_t CHECK_INTERVAL_MS = 250;
constexpr uint32_t PRESS_MS = 1000;
constexpr uint32_t POST_PRESS_COOLDOWN_MS = 5000;
constexpr uint8_t MISMATCH_REQUIRED_COUNT = 2;
} // namespace

MachinePowerSync::MachinePowerSync(uint8_t powerLedSensePin, uint8_t powerButtonPin)
    : powerLedSensePin(powerLedSensePin), powerButtonPin(powerButtonPin) {}

void MachinePowerSync::setup() {
    // Controller PCB has a physical pull-up on this net (/BTN_BREW). Don't enable an internal pulldown here,
    // otherwise you'll create a resistor divider and measure a mid-level voltage.
    pinMode(powerLedSensePin, INPUT);

    digitalWrite(powerButtonPin, LOW);
    pinMode(powerButtonPin, OUTPUT);

    nextCheckMs = 0;
    pressCooldownUntilMs = 0;
    consecutiveMismatchCount = 0;
}

void MachinePowerSync::setDesiredOn(const bool desiredOn) {
    this->desiredOn = desiredOn;
    desiredKnown = true;
}

bool MachinePowerSync::readPowerLed() const {
    // Active-low: LOW means the external sense circuit is pulling the line down (LED line has power).
    return digitalRead(powerLedSensePin) == LOW;
}

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

    // Don't press anything until we've received at least one desired state update from the controller.
    if (!desiredKnown) {
        return;
    }

    if (pressActive) {
        if (timeReached(nowMs, pressStartMs + PRESS_MS)) {
            stopMomentaryPress();
            nextCheckMs = nowMs + CHECK_INTERVAL_MS;
            pressCooldownUntilMs = nowMs + POST_PRESS_COOLDOWN_MS;
            consecutiveMismatchCount = 0;
            ESP_LOGI(LOG_TAG, "Momentary press end");
        }
        return;
    }

    if (!timeReached(nowMs, nextCheckMs)) {
        return;
    }

    nextCheckMs = nowMs + CHECK_INTERVAL_MS;

    const bool powerLedOn = readPowerLed();
    const bool mismatch = desiredOn != powerLedOn;
    if (!mismatch) {
        consecutiveMismatchCount = 0;
        return;
    }

    if (!timeReached(nowMs, pressCooldownUntilMs)) {
        consecutiveMismatchCount = 0;
        return;
    }

    if (consecutiveMismatchCount < 0xFF) {
        ++consecutiveMismatchCount;
    }

    if (consecutiveMismatchCount >= MISMATCH_REQUIRED_COUNT) {
        consecutiveMismatchCount = 0;
        startMomentaryPress(nowMs);
    }
}
