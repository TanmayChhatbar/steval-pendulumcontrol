#include "CustomEncoder.h"
#include <STM32FreeRTOS.h>

CustomEncoder::CustomEncoder(uint8_t pinA, uint8_t pinB, uint32_t ticksPerRotation)
{
    _pinA = pinA;
    _pinB = pinB;
    _resolution = float(ticksPerRotation) / 360.0 * 4.0;
}

void CustomEncoder::begin()
{
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);

    _lastA = digitalRead(_pinA);
    _lastB = digitalRead(_pinB);
    _position = 0;

    static CustomEncoder *instance = this;

    attachInterrupt(digitalPinToInterrupt(_pinA), (voidFuncPtr)[]() { instance->update(); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(_pinB), (voidFuncPtr)[]() { instance->update(); }, CHANGE);

    xTaskCreate(velocityTask, "encoder velocity", 256, this, 2, nullptr);
    xTaskCreate(velocityFilterTask, "encoder velocity filter", 256, this, 2, nullptr);
}

void CustomEncoder::velocityTask(void *arg)
{
    auto *encoder = static_cast<CustomEncoder *>(arg);
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(250);

    for (;;)
    {
        if (micros() - encoder->_lastTimeForVelocity_us > 25000)
        {
            encoder->updateVelocity();
        }
        vTaskDelayUntil(&lastWake, period);
    }
}

void CustomEncoder::velocityFilterTask(void *arg)
{
    auto *encoder = static_cast<CustomEncoder *>(arg);
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1);

    for (;;)
    {
        encoder->updateFilteredVelocity();
        vTaskDelayUntil(&lastWake, period);
    }
}

float CustomEncoder::getPosition() const
{
    // return in degrees
    return float(_position) / _resolution;
}

float CustomEncoder::getVelocityRaw() const
{
    // return in degrees per second
    return _velocity / _resolution;
}

float CustomEncoder::getVelocity() const
{
    // return in degrees per second
    return _filteredVelocity / _resolution;
}

void CustomEncoder::updateFilteredVelocity()
{
    // low-pass filter
    float alpha = LPF;
    _filteredVelocity = alpha * _velocity + (1 - alpha) * _filteredVelocity;
}

void CustomEncoder::update()
{
    // read
    bool currentA = digitalRead(_pinA);
    bool currentB = digitalRead(_pinB);

    // history
    _lastPosition = _position;

    // track
    if (currentA != _lastA || currentB != _lastB)
    {
        if ((currentA ^ _lastB) == 1)
        {
            _position++; // cw
        }
        else
        {
            _position--; // ccw
        }
        _lastA = currentA;
        _lastB = currentB;
    }

    // velocity
    updateVelocity();
}

void CustomEncoder::updateVelocity()
{
    uint32_t currentTime_us = micros();
    uint32_t elapsedTime = currentTime_us - _lastTimeForVelocity_us;
    if (elapsedTime > 0)
    {
        float dt = float(elapsedTime) / 1000000.0;
        _velocity = (_position - _lastPositionForVelocity) / dt;
        _lastPositionForVelocity = _position;
        _lastTimeForVelocity_us = currentTime_us;
    }
}

void CustomEncoder::resetPosition()
{
    _position = 0;
    _lastPosition = 0;
    _lastPositionForVelocity = 0;
    _lastTimeForVelocity_us = 0;
    _velocity = 0.0;
    _filteredVelocity = 0.0;
}
