#include <Arduino.h>
#include "Traintrack.h"

TurnoutDriver driver;

unsigned long refMillis;
unsigned char state = 0;

void handleMovementDone(TurnoutDriver driver, uint8_t channel)
{
    Serial.println();
    Serial.print("Movement done ");
    Serial.println(channel);
}

void setup()
{
    Serial.begin(9600);
    Serial.println("Train Turnout Demo");

    driver.onMovementDone(handleMovementDone);

    if (!driver.begin()) {
        Serial.println("Turnout device not found");
        while(1);
    }

    refMillis = millis();
}

void loop()
{
    driver.loop();

    if(millis() - refMillis > 5000) {
        state++;
        refMillis = millis();

        switch(state) {
            case 1:
                Serial.println("Start move right");
                driver.moveChannelServo(0, TurnoutDriver::TURNOUT_RIGHT);
                break;

            case 2:
                Serial.println("Start move left");
                driver.moveChannelServo(0, TurnoutDriver::TURNOUT_LEFT);
                break;

            case 3:
                Serial.println("End test");
                state = 0;
                break;
        }
    }
}

