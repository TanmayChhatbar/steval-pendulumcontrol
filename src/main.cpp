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
    myStepper.setAcceleration(5000.0f);
    vTaskStartScheduler();
}

void loop()
{
    static unsigned long last_vel_cmd_time = millis();
    while (Serial.available())
    {
        char s = Serial.read();
        if (s == 's')
        {
            float vsp = Serial.parseFloat();
            myStepper.setVelocitySetPoint(vsp);
            last_vel_cmd_time = millis();
        }

        else if (s == 'c')
        {
            float currentLimit = Serial.parseFloat();
            Serial.println(currentLimit);
            myStepper.setCurrentLimit(currentLimit);
        }

        else if (s == 'r')
        {
            myStepper.resetPosition();
            myEnc.resetPosition();
        }
    }

    if (millis() - last_vel_cmd_time > 200)
    {
        // if no command received, zero velocity
        myStepper.setVelocitySetPoint(0.0f);
    }

    myEnc.updateFilteredVelocity();

    Serial.print(myEnc.getPosition());
    Serial.print(",");
    Serial.print(myEnc.getVelocity());
    Serial.print(",");
    Serial.println(myStepper.getPosition());
    delay(10);
}
