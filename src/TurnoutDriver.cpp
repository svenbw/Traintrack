#include "TurnoutDriver.h"

// Register addresses from data sheet
#define PCA9685_MODE1_REG          (uint8_t)0x00
#define PCA9685_MODE2_REG          (uint8_t)0x01
#define PCA9685_SUBADR1_REG        (uint8_t)0x02
#define PCA9685_SUBADR2_REG        (uint8_t)0x03
#define PCA9685_SUBADR3_REG        (uint8_t)0x04
#define PCA9685_ALLCALL_REG        (uint8_t)0x05
#define PCA9685_LED0_REG           (uint8_t)0x06
#define PCA9685_PRESCALE_REG       (uint8_t)0xFE
#define PCA9685_ALLLED_REG         (uint8_t)0xFA

// Mode1 register values
#define PCA9685_MODE1_RESTART      (uint8_t)0x80
#define PCA9685_MODE1_EXTCLK       (uint8_t)0x40
#define PCA9685_MODE1_AUTOINC      (uint8_t)0x20
#define PCA9685_MODE1_SLEEP        (uint8_t)0x10
#define PCA9685_MODE1_SUBADR1      (uint8_t)0x08
#define PCA9685_MODE1_SUBADR2      (uint8_t)0x04
#define PCA9685_MODE1_SUBADR3      (uint8_t)0x02
#define PCA9685_MODE1_ALLCALL      (uint8_t)0x01

// Mode2 register values
#define PCA9685_MODE2_OUTDRV_TPOLE (uint8_t)0x04
#define PCA9685_MODE2_INVRT        (uint8_t)0x10
#define PCA9685_MODE2_OUTNE_TPHIGH (uint8_t)0x01
#define PCA9685_MODE2_OUTNE_HIGHZ  (uint8_t)0x02
#define PCA9685_MODE2_OCH_ONACK    (uint8_t)0x08

#define PCA9685_SW_RESET           (uint8_t)0x06
#define PCA9685_PWM_FULL           (uint16_t)0x1000
#define PCA9685_PWM_MASK           (uint16_t)0x0FFF

#define OSCILLATOR_FREQUENCY       25000000
#define SERVO_FREQUENCY            50
#define SERVO_LEFT                 205
#define SERVO_RIGHT                410
#define RELAY_OFF_STATE            false

#define PCA9685_ALLLED_CHANNEL     -1
#define PCA9685_PRESCALE_MIN       3
#define PCA9685_PRESCALE_MAX       255

TurnoutDriver::TurnoutDriver()
    : _address(0x40), _wire(&Wire), _inCache(0), _outCache(0), _lastError(TURNOUT_DRIVER_OK), _oscillatorFrequency(OSCILLATOR_FREQUENCY), _lastUpdate(0)
{
    commonConstructor();
}

TurnoutDriver::TurnoutDriver(const uint8_t address)
    : _address(address), _wire(&Wire), _inCache(0), _outCache(0), _lastError(TURNOUT_DRIVER_OK), _oscillatorFrequency(OSCILLATOR_FREQUENCY), _lastUpdate(0)
{
    commonConstructor();
}

TurnoutDriver::TurnoutDriver(const uint8_t address, TwoWire *wire)
    : _address(address), _wire(wire), _inCache(0), _outCache(0), _lastError(TURNOUT_DRIVER_OK), _oscillatorFrequency(OSCILLATOR_FREQUENCY), _lastUpdate(0)
{
    commonConstructor();
}

void TurnoutDriver::commonConstructor()
{
    channel_config_t* config;
    uint8_t channel;

    for (channel=0, config=channelConfigs; channel<8; ++channel, ++config) {
        channel_config_t* config = &channelConfigs[channel];
        config->leftPwmValue = SERVO_LEFT;
        config->rightPwmValue = SERVO_RIGHT;

        setChannelDirection(channel, TURNOUT_CENTER);
        setChannel(channel * 2, RELAY_OFF_STATE);
    }
}

#if defined (ESP8266) || defined(ESP32)
bool TurnoutDriver::begin(const int dataPin, const int clockPin)
{
    if ((dataPin > 0) && (clockPin > 0)) {
        _wire->begin(dataPin, clockPin);
    } else {
        _wire->begin();
    }

    return commonBegin();
}
#endif

bool TurnoutDriver::begin()
{
    _wire->begin();

    return commonBegin();
}

bool TurnoutDriver::commonBegin()
{
    reset();

    setPWMFrequency(SERVO_FREQUENCY);

    return true;
}

void TurnoutDriver::reset()
{
    writeDeviceReset();
    if (_lastError != 0) {
        return;
    }

    writeRegister(PCA9685_MODE1_REG, PCA9685_MODE1_RESTART | PCA9685_MODE1_AUTOINC);
    if (_lastError != 0) {
        return;
    }

    writeRegister(PCA9685_MODE2_REG, PCA9685_MODE2_OUTDRV_TPOLE);
    delayMicroseconds(1000);
}

void TurnoutDriver::setOscillatorFrequency(const uint32_t frequency)
{
    _oscillatorFrequency = frequency;
}

void TurnoutDriver::setPWMFrequency(float frequency)
{
    if (frequency < 1)
        frequency = 1;
    if (frequency > 3500)
        frequency = 3500; // Datasheet limit is 3052=50MHz/(4*4096)

    float prescaleval = ((_oscillatorFrequency / (frequency * 4096.0)) + 0.5) - 1;
    if (prescaleval < PCA9685_PRESCALE_MIN) {
        prescaleval = PCA9685_PRESCALE_MIN;
    }
    if (prescaleval > PCA9685_PRESCALE_MAX) {
        prescaleval = PCA9685_PRESCALE_MAX;
    }
    uint8_t prescale = (uint8_t)prescaleval;

    uint8_t oldmode = readRegister(PCA9685_MODE1_REG);
    uint8_t newmode = (oldmode & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP;
    writeRegister(PCA9685_MODE1_REG, newmode);
    writeRegister(PCA9685_PRESCALE_REG, prescale);
    writeRegister(PCA9685_MODE1_REG, (oldmode & ~PCA9685_MODE1_SLEEP) | PCA9685_MODE1_RESTART);
    delayMicroseconds(500);
}

void TurnoutDriver::setChannelRanges(const uint8_t channel, const uint16_t leftPwmValue, const uint16_t rightPwmValue)
{
    if (channel > 7)  {
        return;
    }
    channel_config_t* config = &channelConfigs[channel];

    config->leftPwmValue = leftPwmValue;
    config->rightPwmValue = rightPwmValue;
}

void TurnoutDriver::setChannelDirection(const uint8_t channel, const turnout_direction_e direction)
{
    if (channel > 7)  {
        return;
    }

    channel_state_t* state = &channelStates[channel];
    if (state->value != state->targetValue) {
        return;
    }

    channel_config_t* config = &channelConfigs[channel];
    switch(direction) {
        case TURNOUT_LEFT:
            state->value = config->leftPwmValue;
            break;

        case TURNOUT_RIGHT:
            state->value = config->rightPwmValue;
            break;

        case TURNOUT_CENTER:
            if (config->rightPwmValue > config->leftPwmValue) {
                state->value = config->rightPwmValue - config->leftPwmValue;
            } else {
                state->value = config->leftPwmValue - config->rightPwmValue;
            }
            break;
    }

    state->target = direction;
    state->targetValue = state->value;
    state->step = 0;

    setChannelPWM(1 + channel * 2, state->value);
}

void TurnoutDriver::onMovementDone(CallbackFunction cb)
{
    movementDoneCb = cb;
}

void TurnoutDriver::moveChannelServo(const uint8_t channel, const turnout_direction_e direction)
{
    if (channel > 7)  {
        return;
    }

    if (direction == TURNOUT_CENTER) {
        return;        
    }

    int16_t dir;
    channel_config_t* config = &channelConfigs[channel];
    if (config->rightPwmValue > config->leftPwmValue) {
        dir = 1;
    } else {
        dir = -1;
    }

    channel_state_t* state = &channelStates[channel];
    switch(direction) {
        case TURNOUT_LEFT:
            state->step = 10 * -dir;
            state->targetValue = config->leftPwmValue;
            break;

        case TURNOUT_RIGHT:
            state->step = 10 * dir;
            state->targetValue = config->rightPwmValue;
            break;

        default:
            return;
    }

    state->target = direction;
}

void TurnoutDriver::loop()
{
    channel_config_t* config;
    channel_state_t* state;
    uint8_t channel;

    if (millis() <= _lastUpdate + 25) {
        return;
    }

    for (channel=0, config=channelConfigs, state=channelStates; channel<8; ++channel, ++config, ++state) {
        if (state->step == 0) {
            continue;
        }

        state->value += state->step;
        if ((state->step > 0 && state->value > state->targetValue) || (state->step < 0 && state->value < state->targetValue)) {
            state->value = state->targetValue;
            state->step = 0;
        }
        setChannelPWM(1 + 2 * channel, state->value);

        if (state->step == 0) {
            if (movementDoneCb) {
                movementDoneCb(*this, channel);
            }
        }
    }

    _lastUpdate = millis();
}

void TurnoutDriver::setChannelPWM(const uint8_t pwmChannel, const uint16_t pwm)
{
    if (pwm == 0 || pwm >= PCA9685_PWM_FULL)  {
        setChannel(pwmChannel, pwm > 0);
        return;
    }

    writeChannelPhases(pwmChannel, 0, pwm & PCA9685_PWM_MASK);
}

void TurnoutDriver::setChannel(const uint8_t pwmChannel, const bool state)
{
    if (state) {
        writeChannelPhases(pwmChannel, PCA9685_PWM_FULL, 0);
    } else {
        writeChannelPhases(pwmChannel, 0, PCA9685_PWM_FULL);
    }
}

void TurnoutDriver::writeDeviceReset()
{
    _wire->beginTransmission(_address);
    _wire->write(PCA9685_SW_RESET);
    _lastError = _wire->endTransmission();
    delayMicroseconds(10);
}

void TurnoutDriver::writeRegister(const uint8_t regAddress, const uint8_t data)
{
    _wire->beginTransmission(_address);
    _wire->write(regAddress);
    _wire->write(data);
    _lastError = _wire->endTransmission();
}

uint8_t TurnoutDriver::readRegister(const uint8_t regAddress)
{
    _wire->beginTransmission(_address);
    _wire->write(regAddress);
    _lastError = _wire->endTransmission();
    if (_lastError) {
        return 0;
    }

    if (_wire->requestFrom(_address, (uint8_t)1) != 1) {
        _lastError = TURNOUT_DRIVER_I2C_ERROR;
        return _inCache;
    }

    _inCache = _wire->read() & 0xFF;

    _lastError = TURNOUT_DRIVER_OK;

    return _inCache;
}

void TurnoutDriver::writeChannelPhases(const uint8_t pwmChannel, uint16_t phaseBegin, uint16_t phaseEnd)
{
    uint8_t regAddress;

    regAddress = PCA9685_LED0_REG + (pwmChannel * 4);

    _wire->beginTransmission(_address);
    _wire->write(regAddress);

    _wire->write(lowByte(phaseBegin));
    _wire->write(highByte(phaseBegin));
    _wire->write(lowByte(phaseEnd));
    _wire->write(highByte(phaseEnd));

    _lastError = _wire->endTransmission();
}

bool TurnoutDriver::isMoving(const uint8_t channel)
{
    if (channel > 7)  {
        return false;
    }

    return channelStates[channel].step != 0;
}

int TurnoutDriver::lastError()
{
    int error = _lastError;
    _lastError = TURNOUT_DRIVER_OK;

    return error;
}
