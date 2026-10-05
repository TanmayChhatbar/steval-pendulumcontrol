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
    myStepper.setAcceleration(10000.0f);
    vTaskStartScheduler();
}

void loop()
{
    static unsigned long last_vel_cmd_time = millis();
    if (Serial.available())
    {
        char s = Serial.read();
        if (s == 's')
        {
            myStepper.setVelocitySetPoint(Serial.parseFloat());
            last_vel_cmd_time = millis();
        }

        else if (s == 'c')
        {
            myStepper.setCurrentLimit(Serial.parseFloat());
        }
    }
    if (millis() - last_vel_cmd_time > 200)
    {
        myStepper.setVelocitySetPoint(0.0f);
    }

    myEnc.updateFilteredVelocity();

    static unsigned long startTime = millis();
    unsigned long currentTime = millis();
    unsigned long elapsed = currentTime - startTime;

    Serial.print(myEnc.getPosition());
    Serial.print(",");
    Serial.println(myEnc.getVelocity());

    if (elapsed >= 1000)
    {
        startTime = currentTime;
    }
    delay(10);
}
