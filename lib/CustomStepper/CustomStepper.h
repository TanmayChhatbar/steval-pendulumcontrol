#include <Arduino.h>

class CustomStepper
{
public:
    CustomStepper(uint8_t pinDir, uint8_t pinStep, float degreesPerTick);
    float getPosition() const;
    void setVelocitySetPoint(float velocity);
    void begin();
    void setAcceleration(float accMax);
    void setCurrentLimit(float currentLimit);
    void resetPosition();

private:
    static void controlTask(void *arg);
    uint8_t _pinDir;
    uint8_t _pinStep;
    float _velocitySetpointFromUser = 0.0;
    float _velocity = 0.0;
    float _accMax = 10.0;
    float _resolution; // degrees per tick
    uint32_t _lastControlTimeUs = 0;
    uint32_t _lastStepRate = 0;
    bool _outputForward = false;
};
