#include <Arduino.h>
#include <functional>

// define as input pullup
class CustomEncoder {
public:
    CustomEncoder(uint8_t pinA, uint8_t pinB, uint32_t ticksPerRotation) : _pinA(pinA), _pinB(pinB) {
        pinMode(_pinA, INPUT_PULLUP);
        pinMode(_pinB, INPUT_PULLUP);

        _lastA = digitalRead(_pinA);
        _lastB = digitalRead(_pinB);
        _position = 0;
        _resolution = float(ticksPerRotation) / 360.0;

        attachInterrupt(digitalPinToInterrupt(_pinA), std::bind(&CustomEncoder::update, this), CHANGE);
    }
    float getPosition() const {
        // return in degrees
        return float(_position) / _resolution;
    }
    float getVelocityRaw() const {
        // return in degrees per second
        return _velocity / _resolution;
    }
    float getVelocity() const {
        // return in degrees per second
        return _filteredVelocity / _resolution;
    }

private:
    uint8_t _pinA, _pinB;
    int32_t _lastPosition = 0;
    int32_t _position = 0;
    bool _lastA;
    bool _lastB;
    uint32_t _lastTime = 0;
    float _velocity = 0.0;
    float _filteredVelocity = 0.0;
    float _resolution; // ticks per degree
    
    void update() {
        // read
        bool currentA = digitalRead(_pinA);
        _lastB = digitalRead(_pinB);
        
        // track
        if (currentA != _lastA) {
            _lastPosition = _position;
            if (currentA == _lastB) {
                _position--;  // ccw
            } else {
                _position++;  // cw
            }
            _lastA = currentA;
        }

        // velocity
        uint32_t currentTime = millis();
        uint32_t elapsedTime = currentTime - _lastTime;
        if (elapsedTime > 0) {
            _velocity = (_position - _lastPosition) / float(elapsedTime) * 1000.0;
        }
        _lastTime = currentTime;
    }
};
