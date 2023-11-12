#pragma once

#include <Arduino.h>
#include <Wire.h>

#define TURNOUT_DRIVER_OK 0x00
#define TURNOUT_DRIVER_PIN_ERROR 0x81
#define TURNOUT_DRIVER_I2C_ERROR 0x82

class TurnoutDriver
{
    public:
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP8266)
        typedef std::function<void(TurnoutDriver &driver, uint8_t channel)> CallbackFunction;
#else
        typedef void (*CallbackFunction)(TurnoutDriver &driver, uint8_t channel);
#endif
        typedef enum {
            TURNOUT_CENTER,
            TURNOUT_LEFT,
            TURNOUT_RIGHT,
        } turnout_direction_e;

    private:
        void commonConstructor();
        bool commonBegin();
        void reset();
        void writeDeviceReset();
        void setOscillatorFrequency(uint32_t frequency);

        void setChannelPWM(const uint8_t channel, const uint16_t pwm);

        void writeChannelPhases(const uint8_t channel, uint16_t phaseBegin, uint16_t phaseEnd);

        void writeRegister(const uint8_t regAddress, const uint8_t data);
        uint8_t readRegister(const uint8_t regAddress);

        typedef struct {
            uint16_t leftPwmValue;
            uint16_t rightPwmValue;
        } channel_config_t;

        typedef struct {
            turnout_direction_e target;
            uint16_t value;
            uint16_t targetValue;
            int16_t step;
        } channel_state_t;

        uint8_t _address;
        TwoWire*  _wire;
        uint8_t _inCache;
        uint8_t _outCache;
        int _lastError;
        uint32_t _oscillatorFrequency;
        unsigned long _lastUpdate;
        CallbackFunction movementDoneCb = NULL;

        channel_config_t channelConfigs[8];
        channel_state_t channelStates[8];

    public:
        TurnoutDriver();
        TurnoutDriver(const uint8_t address);
        TurnoutDriver(const uint8_t address, TwoWire *wire);
        void setPWMFrequency(float frequency);
        void setChannelRanges(const uint8_t channel, const uint16_t leftPwmValue, const uint16_t rightPwmValue);
        void setChannelDirection(const uint8_t channel, const turnout_direction_e direction);

        void onMovementDone(CallbackFunction cb);

        int lastError();
        bool isMoving(const uint8_t channel);

        void moveChannelServo(const uint8_t channel, const turnout_direction_e direction);
        void setChannel(const uint8_t channel, const bool state);

        void loop();

#if defined (ESP8266) || defined(ESP32)
        bool begin(const int sda, const int scl);
#endif
        bool begin();
};