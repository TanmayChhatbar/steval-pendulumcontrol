#include <Arduino.h>
#include "CustomEncoder.h"
#include "CustomStepper.h"
#include <STM32FreeRTOS.h>

CustomEncoder myEnc(D5, D4, 600U);
CustomStepper myStepper(D7, D9, 1.8f / 16.0f);
double start_time_ms;
void serialTask(void *pvParameters)
{
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);
    for (;;)
    {
        Serial.print(((double)(micros()) / 1000.0 - start_time_ms) / 1000.0, 3);
        Serial.print(",");
        Serial.print(myEnc.getPosition());
        Serial.print(",");
        Serial.print(myEnc.getVelocity());
        Serial.print(",");
        Serial.println(myStepper.getPosition());
        Serial.flush();

        vTaskDelayUntil(&lastWake, period);
    }
}

void setup()
{
    Serial.begin(921600);
    delay(2000);
    myEnc.begin();
    myStepper.begin();
    Serial.println(myStepper.getPosition());
    delay(1000);
    myStepper.setAcceleration(5000.0f);
    xTaskCreate(serialTask, "Serial Task", 256, NULL, 1, NULL);
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
            start_time_ms = micros() / 1000.0;
        }
    }

    if (millis() - last_vel_cmd_time > 200)
    {
        // if no command received, zero velocity
        myStepper.setVelocitySetPoint(0.0f);
    }
}
