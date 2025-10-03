#include <Arduino.h>
#include "CustomEncoder.h"

CustomEncoder myEnc(PB4, PB5, 600U);

void setup() {
    Serial.begin(230400);
}

void loop() {
    static unsigned long startTime = millis();
    unsigned long currentTime = millis();
    unsigned long elapsed = currentTime - startTime;

    Serial.print(elapsed);
    Serial.print(",");
    Serial.print(myEnc.getPosition());
    Serial.print(",");
    Serial.println(myEnc.getVelocity());

    if (elapsed >= 1000) {
        startTime = currentTime;
    }
    delay(10);
}
