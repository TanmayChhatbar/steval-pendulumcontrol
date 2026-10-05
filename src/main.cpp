#include <Arduino.h>
#include "CustomEncoder.h"
// #include <STM32FreeRTOS.h>

CustomEncoder myEnc(D5, D4, 600U);

// void updateFilteredVelocity(void *pvParameters)
// {
//     const TickType_t xDelay = pdMS_TO_TICKS(10); // 10 ms delay
//     for (;;)
//     {
//         myEnc.updateFilteredVelocity();
//     }
//     vTaskDelay(xDelay);
// }

void setup()
{
    Serial.begin(230400);
    myEnc.begin();
    // xTaskCreate(
    //     updateFilteredVelocity,
    //     "UpdateFilteredVelocity",
    //     2048,
    //     NULL,
    //     1,
    //     NULL);

    // pinMode(D4, INPUT_PULLUP);
    // pinMode(D5, INPUT_PULLUP);
    // uint lastState = 0b00;
    // while (true)
    // {
    //     uint newState = (digitalRead(D5) << 1) | digitalRead(D4);
    //     if (newState != lastState)
    //     {
    //         lastState = newState;
    //         Serial.println(String(lastState & 0b01) + String(lastState & 0b10 ? 1 : 0));
    //     }
    //     delay(1);
    // }
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
    // myEnc.update();
    myEnc.updateFilteredVelocity();
}
