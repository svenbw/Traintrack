#include <Arduino.h>
#include "Traintrack.h"

TurnoutController controller;

bool handleTurnLeft(TurnoutController controller, uint8_t channel)
{
    Serial.println();
    Serial.print("Turn left on channel ");
    Serial.println(channel);

    return true;
}

bool handleTurnRight(TurnoutController controller, uint8_t channel)
{
    Serial.print("\nTurn right on channel ");
    Serial.println(channel);

    return true;
}

void setup()
{
    Serial.begin(9600);
    Serial.println("Train Controller Demo");

    controller.onTurnLeft(handleTurnLeft);
    controller.onTurnRight(handleTurnRight);

    if (!controller.begin()) {
        Serial.println("Controller device not found");
        while(1);
    }
}

void loop()
{
    controller.loop();
}

