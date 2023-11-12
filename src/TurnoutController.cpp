#include "TurnoutController.h"

#define BUTTON_MASK 0b0100100100100100
#define SINGLE_BUTTON_MASK 0b100
#define LEFT_LED_MASK 0b001
#define RIGHT_LED_MASK 0b010
#define DEBOUNCE_DELAY 50

TurnoutController::TurnoutController()
    : _address(0x20), _wire(&Wire), _inCache(0), _outCache(0), _lastError(TRAIN_CONTROLLER_OK), _lastUpdate(0), _enabledChannels(31)
{
    commonConstructor();
}

TurnoutController::TurnoutController(const uint8_t address)
    : _address(address), _wire(&Wire), _inCache(0), _outCache(0), _lastError(TRAIN_CONTROLLER_OK), _lastUpdate(0), _enabledChannels(31)
{
    commonConstructor();
}

TurnoutController::TurnoutController(const uint8_t address, TwoWire *wire)
    : _address(address), _wire(wire), _inCache(0), _outCache(0), _lastError(TRAIN_CONTROLLER_OK), _lastUpdate(0), _enabledChannels(31)
{
    commonConstructor();
}

void TurnoutController::commonConstructor()
{
    for (uint8_t channel=0; channel<4; channel++) {
        _turnoutState[channel] = TURNOUT_OPEN;
    }
}

#if defined (ESP8266) || defined(ESP32)
bool TurnoutController::begin(const int dataPin, const int clockPin)
{
    if ((dataPin > 0) && (clockPin > 0)) {
        _wire->begin(dataPin, clockPin);
    } else {
        _wire->begin();
    }

    return commonBegin();
}
#endif

bool TurnoutController::begin()
{
    _wire->begin();

    return commonBegin();
}

bool TurnoutController::commonBegin()
{
    if (! isConnected()) {
        return false;
    }

    _lastButtonState = read();
    _lastUpdate = millis();

    return true;
}

bool TurnoutController::isConnected()
{
    _wire->beginTransmission(_address);
    return (_wire->endTransmission() == 0);
}

void TurnoutController::onTurnLeft(CallbackFunction cb)
{
    turnLeftCb = cb;
}

void TurnoutController::onTurnRight(CallbackFunction cb)
{
    turnRightCb = cb;
}

void TurnoutController::onToggle(CallbackFunction cb)
{
    toggleCb = cb;
}

void TurnoutController::setEnabledChannels(uint8_t channels)
{
    _enabledChannels = channels;
}

bool TurnoutController::isDirectionLeft(uint8_t channel)
{
    return _turnoutState[channel] == TURNOUT_LEFT;
}

bool TurnoutController::isDirectionRight(uint8_t channel)
{
    return _turnoutState[channel] == TURNOUT_RIGHT;
}

TurnoutController::turnout_state_e TurnoutController::getNextDirection(uint8_t channel)
{
    switch(_turnoutState[channel]) {
        case TURNOUT_LEFT:
            return TURNOUT_RIGHT;
            break;

        case TURNOUT_RIGHT:
            return TURNOUT_LEFT;
            break;

        default:
            // Allow setting of defaults?
            return TURNOUT_LEFT;
    }
}

void TurnoutController::loop()
{
    readButtons();
    updateDirections();
    updateLEDs();
}

void TurnoutController::readButtons()
{
    uint16_t temp = _outCache;

    write(_outCache | BUTTON_MASK);
    read();
    write(temp);
}

void TurnoutController::updateDirections()
{
    uint16_t value = _inCache & BUTTON_MASK;
    uint16_t diff = _lastButtonState ^ value;
    if ((diff == 0) || (millis() - _lastUpdate < DEBOUNCE_DELAY)) {
        return;
    }

    for (uint8_t channel=0; channel<4; channel++) {
        if ((_enabledChannels & (1 << channel)) && (value & SINGLE_BUTTON_MASK) == 0) {
            turnout_state_e nextDirection = getNextDirection(channel);
            bool allowTurn = true;

            if (nextDirection == TURNOUT_LEFT) {
                if (turnLeftCb) {
                    allowTurn &= turnLeftCb(*this, channel);
                }
            }
            else if (nextDirection == TURNOUT_RIGHT) {
                if (turnRightCb) {
                    allowTurn &= turnRightCb(*this, channel);
                }
            }

            if (toggleCb) {
                allowTurn &= toggleCb(*this, channel);
            }

            if (allowTurn) {
                _turnoutState[channel] = nextDirection;
            }
        }

        value >>= 3;
        diff >>= 3;
    }

    _lastButtonState = _inCache & BUTTON_MASK;
    _lastUpdate = millis();
}

void TurnoutController::updateLEDs()
{
    uint16_t value = _outCache;
    uint16_t mask = LEFT_LED_MASK | RIGHT_LED_MASK;

    for (uint8_t channel=0; channel<4; channel++) {
        if (_turnoutState[channel] == TURNOUT_LEFT) {
            value &= ~mask;
            value |= RIGHT_LED_MASK << (3*channel);
        } else if (_turnoutState[channel] == TURNOUT_RIGHT) {
            value &= ~mask;
            value |= LEFT_LED_MASK << (3*channel);
        }

        mask <<= 3;
    }
    if (value != _outCache) {
        write(value);
    }
}

uint16_t TurnoutController::read()
{
    if (_wire->requestFrom(_address, (uint8_t)2) != 2) {
        _lastError = TRAIN_CONTROLLER_I2C_ERROR;
        return _inCache;
    }

    _inCache = _wire->read();
    _inCache |= (_wire->read() << 8);

    _lastError = TRAIN_CONTROLLER_OK;

    return _inCache;
}

void TurnoutController::write(const uint16_t value)
{
    _outCache = value;
    _wire->beginTransmission(_address);
    _wire->write(_outCache & 0xFF);
    _wire->write(_outCache >> 8);
    _lastError = _wire->endTransmission();
}

int TurnoutController::lastError()
{
    int error = _lastError;
    _lastError = TRAIN_CONTROLLER_OK;

    return error;
}
