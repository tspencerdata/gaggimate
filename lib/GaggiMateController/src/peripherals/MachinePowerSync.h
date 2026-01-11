#ifndef MACHINEPOWERSYNC_H
#define MACHINEPOWERSYNC_H

#include <Arduino.h>

class MachinePowerSync {
  public:
    MachinePowerSync(uint8_t powerLedSensePin, uint8_t powerButtonPin);
    ~MachinePowerSync() = default;

    void setup();
    void loop();

    void setDesiredOn(bool desiredOn);

  private:
    uint8_t powerLedSensePin;
    uint8_t powerButtonPin;

    bool desiredOn = false;
    bool pressActive = false;
    uint32_t pressStartMs = 0;
    uint32_t nextCheckMs = 0;

    bool readPowerLed() const;
    void startMomentaryPress(uint32_t nowMs);
    void stopMomentaryPress();

    static bool timeReached(uint32_t nowMs, uint32_t targetMs);

    const char *LOG_TAG = "MachinePowerSync";
};

#endif // MACHINEPOWERSYNC_H
