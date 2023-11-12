#pragma once

#include <Arduino.h>
#include <Wire.h>

#define TRAIN_CONTROLLER_OK 0x00
#define TRAIN_CONTROLLER_PIN_ERROR 0x81
#define TRAIN_CONTROLLER_I2C_ERROR 0x82

#define CONTROL_CHANNEL_0 1
#define CONTROL_CHANNEL_1 2
#define CONTROL_CHANNEL_2 4
#define CONTROL_CHANNEL_3 8
#define CONTROL_CHANNEL_4 16

class TurnoutController
{
    private:
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP8266)
        typedef std::function<bool(TurnoutController &controller, uint8_t channel)> CallbackFunction;
#else
        typedef bool (*CallbackFunction)(TurnoutController &controller, uint8_t channel);
#endif
        typedef enum {
            TURNOUT_OPEN,    
            TURNOUT_LEFT,
            TURNOUT_RIGHT,
        } turnout_state_e;

        void commonConstructor();
        bool commonBegin();
        void write(const uint16_t value);
        uint16_t read();
        turnout_state_e getNextDirection(uint8_t channel);
        void readButtons();
        void updateDirections();
        void updateLEDs();

        uint8_t _address;
        TwoWire*  _wire;
        uint16_t _inCache;
        uint16_t _outCache;
        int _lastError;
        unsigned long _lastUpdate;
        uint8_t _enabledChannels;
        uint16_t _lastButtonState;
        turnout_state_e _turnoutState[5];

        CallbackFunction turnLeftCb = NULL;
        CallbackFunction turnRightCb = NULL;
        CallbackFunction toggleCb = NULL;

    public:
        TurnoutController();
        TurnoutController(const uint8_t address);
        TurnoutController(const uint8_t address, TwoWire *wire);

#if defined (ESP8266) || defined(ESP32)
        bool begin(const int sda, const int scl);
#endif
        bool begin();
        bool isConnected();
        int lastError();

        void onTurnLeft(CallbackFunction f);
        void onTurnRight(CallbackFunction f);
        void onToggle(CallbackFunction f);
        void setEnabledChannels(uint8_t channels);

        void loop();
        bool isDirectionLeft(uint8_t channel);
        bool isDirectionRight(uint8_t channel);
};
