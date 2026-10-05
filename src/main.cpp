#include <Arduino.h>
#include "CustomEncoder.h"
#include <STM32FreeRTOS.h>

CustomEncoder myEnc(D5, D4, 600U);

void setup()
{
    Serial.begin(230400);
    myEnc.begin();
    vTaskStartScheduler();
}

void loop()
{
    static unsigned long startTime = millis();
    unsigned long currentTime = millis();
    unsigned long elapsed = currentTime - startTime;

    Serial.print(elapsed);
    Serial.print("\t");
    Serial.print(myEnc.getPosition());
    Serial.print("\t");
    Serial.println(myEnc.getVelocityRaw());

    if (elapsed >= 1000)
    {
        startTime = currentTime;
    }
    delay(10);
    myEnc.updateFilteredVelocity();
}
