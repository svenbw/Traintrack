[![Arduino-lint](https://github.com/svenbw/Traintrack/actions/workflows/arduino-lint.yml/badge.svg)](https://github.com/svenbw/Traintrack/actions/workflows/arduino-lint.yml)
[![GitHub issues](https://img.shields.io/github/issues/svenbw/Traintrack.svg)](https://github.com/svenbw/Traintrack/issues)

[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](https://github.com/svenbw/Traintrack/blob/master/LICENSE)


# TrainTrack

Arduino/ESP library to control model train turnouts.

It uses two types of I2C io-expanders.
- PCA9685 for to control the turnouts servo's and the relais.
- PCF8575 for the control panel.


## Description

This library facilitates the management of turnouts on model train tracks
through two classes. The first, `TurnoutController`, employs the PCF8575 for
control panel operations. The second class, `TurnoutDriver`, utilizes the
PCA9685 to control the turnouts.


### PCA9685

The PCA9685 port pins are paired in groups of two: the first pin is used
for PWM-based servo control, while the second one is responsible to switch
the relay.

First 2 channel mapping:

|  port pin  |  channel  |  purpose          |
|:-----------|:----------|:------------------|
|  P0        |  0        |  Turnout 0 relay  |
|  P1        |  0        |  Turnout 0 servo  |
|  P2        |  1        |  Turnout 1 relay  |
|  P3        |  1        |  Turnout 1 servo  |
...

### PCF8575

The PCF8575 port pins are organized in groups of three: the first two are
configured as outputs, responsible for controlling the left and right
indicator LEDs, while the third functions as an input, dedicated to the 
push button.
Each time the push button is pressed, the direction is toggled, triggering
a callback function. This feature allows the control class to be utilized
not only for directional switching but also as a straightforward on/off
switch.

First 2 channel mapping:

|  port pin  |  channel  |  purpose         |
|:-----------|:----------|:-----------------|
|  P0        |  0        |  Button 0        |
|  P1        |  0        |  Left LED 0      |
|  P2        |  0        |  Right LED 0     |
|  P3        |  1        |  Button 1        |
|  P4        |  1        |  Left LED 1      |
|  P5        |  1        |  Right LED 1     |
...


## How To Use

This library provides the flexibility to use the `TurnoutController` and
`TurnoutDriver` classes independently. It is however possible to include
both classes simultaneously through the `TrainTrack.h` header.
The library's inputs and outputs are updated within the main `loop()`
function by invoking the `loop()` function on each class instance.


### Definition

- Include the library on top

```c++
 #include "TrainTrack.h"
```

- Define the class either using the `constructor` or the `begin()` function.

```c++
    TurnoutController controller;
    TurnoutDriver driver;
```

- In the `setup()` function the callback functions can be linked to the
  events from the classes.

```c++
    controller.onTurnLeft(handleTurnLeft);
    controller.onTurnRight(handleTurnRight);

    driver.onMovementDone(handleMovementDone);
```


### Callback Handlers

- The callback handler of the `TurnoutController` returns the controller instance
  and the channel and is invoked if the button toggled the direction
- The return value should reflect if the state of the indicator LED's should
  change.

```c++
bool handleTurnLeft(TurnoutController controller, uint8_t channel)
{
    Serial.print("Turn left on channel ");
    Serial.println(channel);

    return true;
}
```

- The callback handler of the `TurnoutDriver` returns the controller instance
  and the channel and is invoked once the movement is finished.

```c++
void handleMovementDone(TurnoutDriver controller, uint8_t channel)
{
    Serial.print("Movement done on channel ");
    Serial.println(channel);

    return true;
}
```


### The Loop

- For the classes to work, one needs to call the button's `loop()` member
  function in the sketch's `loop()` function.

```c++
void loop()
{
    controller.loop();
    driver.loop();
}
```


### Control functions

- The turnouts can be moved by calling the `moveChannelServo()` member function.
  The fist argument takes channel to control, the second argument is the direction.

```c++
driver.moveChannelServo(0, TurnoutDriver::TURNOUT_RIGHT);
```

