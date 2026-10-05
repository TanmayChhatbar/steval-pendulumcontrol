#include <Arduino.h>
#include "CustomEncoder.h"
#include "CustomStepper.h"
#include <STM32FreeRTOS.h>

CustomEncoder myEnc(D5, D4, 600U);
CustomStepper myStepper(D7, D9, 1.8f / 16.0f);

void setup()
{
    Serial.begin(921600);
    delay(2000);
    myEnc.begin();
    myStepper.begin();
    Serial.println(myStepper.getPosition());
    delay(1000);
    myStepper.setAcceleration(8000.0f);
    vTaskStartScheduler();
}

void loop()
{
    static uint mode = 0;
    static unsigned long last_time = millis();
    static unsigned long delayTime = 500;
    if (millis() - last_time >= delayTime)
    {
        switch (mode)
        {
        case 0:
            myStepper.setVelocitySetPoint(180.0f);
            delayTime = 1000;
            break;
        case 1:
            myStepper.setVelocitySetPoint(0.0f);
            delayTime = 500;
            break;
        case 2:
            myStepper.setVelocitySetPoint(0.0f);
            delayTime = 200;
            break;
        case 3:
            myStepper.setVelocitySetPoint(0.0f);
            delayTime = 500;
            break;
        }
        Serial.println(mode);
        mode++;
        mode %= 4;

        last_time = millis();
    }

    myEnc.updateFilteredVelocity();

    static unsigned long startTime = millis();
    unsigned long currentTime = millis();
    unsigned long elapsed = currentTime - startTime;

    Serial.print(elapsed);
    Serial.print("\t");
    Serial.print(myEnc.getPosition());
    Serial.print("\t");
    Serial.print(myStepper.getPosition());
    Serial.print("\t");
    Serial.println(myEnc.getVelocity());

    if (elapsed >= 1000)
    {
        startTime = currentTime;
    }
    delay(10);
}
