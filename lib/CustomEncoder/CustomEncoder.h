#include <Arduino.h>
#include <functional>

#define Ts 0.01
#define rise_time 0.1
#define LPF (1 - exp(-Ts / rise_time))

// define as input pullup
class CustomEncoder
{
public:
    CustomEncoder(uint8_t pinA, uint8_t pinB, uint32_t ticksPerRotation);
    float getPosition() const;
    float getVelocityRaw() const;
    float getVelocity() const;
    void updateFilteredVelocity();
    void begin();
    void resetPosition();

private:
    static void velocityTask(void *arg);
    void update();
    void updateVelocity();

    uint8_t _pinA, _pinB;
    int32_t _lastPosition = 0;
    int32_t _lastPositionForVelocity = 0;
    uint32_t _lastTimeForVelocity_us = 0;
    int32_t _position = 0;
    bool _lastA;
    bool _lastB;
    uint32_t _lastTime_us = 0;
    float _velocity = 0.0;
    float _filteredVelocity = 0.0;
    float _resolution; // ticks per degree
};
