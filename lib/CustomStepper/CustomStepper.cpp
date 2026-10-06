#include "CustomStepper.h"

#include <SPI.h>
#include <STM32FreeRTOS.h>
#include <math.h>

#define DT 0.001f

namespace
{
    constexpr uint8_t L6474_RESET_PIN = 8;
    constexpr uint8_t L6474_CS_PIN = SS;
    constexpr uint8_t L6474_STEP_MODE_REG = 0x16;
    constexpr uint8_t L6474_TVAL_REG = 0x09;
    constexpr uint8_t L6474_T_FAST_REG = 0x0E;
    constexpr uint8_t L6474_TON_MIN_REG = 0x0F;
    constexpr uint8_t L6474_TOFF_MIN_REG = 0x10;
    constexpr uint8_t L6474_OCD_TH_REG = 0x13;
    constexpr uint8_t L6474_ALARM_EN_REG = 0x17;
    constexpr uint8_t L6474_CONFIG_REG = 0x18;
    constexpr uint8_t L6474_ABS_POS_REG = 0x01;
    constexpr uint8_t L6474_GET_PARAM = 0x20;
    constexpr uint8_t L6474_GET_STATUS = 0xD0;
    constexpr uint8_t L6474_ENABLE = 0xB8;
    constexpr uint8_t L6474_DISABLE = 0xA8;
    constexpr uint16_t L6474_MAX_STEP_RATE = 10000;
    constexpr uint32_t CONTROL_PERIOD_MS = uint32_t(DT * 1000);
    constexpr uint32_t SPI_FREQUENCY_HZ = 1000000;

    uint8_t transferByte(uint8_t value)
    {
        digitalWrite(L6474_CS_PIN, LOW);
        uint8_t received = SPI.transfer(value);
        digitalWrite(L6474_CS_PIN, HIGH);
        return received;
    }

    void writeRegister(uint8_t reg, uint32_t value, uint8_t bytes)
    {
        SPI.beginTransaction(SPISettings(SPI_FREQUENCY_HZ, MSBFIRST, SPI_MODE3));
        transferByte(reg);
        while (bytes > 0)
        {
            --bytes;
            transferByte(static_cast<uint8_t>(value >> (bytes * 8)));
        }
        SPI.endTransaction();
    }

    uint32_t readRegister(uint8_t reg, uint8_t bytes)
    {
        uint32_t value = 0;
        SPI.beginTransaction(SPISettings(SPI_FREQUENCY_HZ, MSBFIRST, SPI_MODE3));
        transferByte(L6474_GET_PARAM | reg);
        while (bytes > 0)
        {
            value = (value << 8) | transferByte(0);
            --bytes;
        }
        SPI.endTransaction();
        return value;
    }

    void sendCommand(uint8_t command)
    {
        SPI.beginTransaction(SPISettings(SPI_FREQUENCY_HZ, MSBFIRST, SPI_MODE3));
        transferByte(command);
        SPI.endTransaction();
    }

    void clearStatus()
    {
        SPI.beginTransaction(SPISettings(SPI_FREQUENCY_HZ, MSBFIRST, SPI_MODE3));
        transferByte(L6474_GET_STATUS);
        transferByte(0);
        transferByte(0);
        SPI.endTransaction();
    }
}

CustomStepper::CustomStepper(uint8_t pinDir, uint8_t pinStep, float degreesPerTick)
    : _pinDir(pinDir), _pinStep(pinStep), _resolution(degreesPerTick)
{
}

void CustomStepper::begin()
{
    pinMode(_pinDir, OUTPUT);
    pinMode(_pinStep, OUTPUT);
    digitalWrite(_pinDir, LOW);
    digitalWrite(_pinStep, LOW);

    pinMode(L6474_CS_PIN, OUTPUT);
    digitalWrite(L6474_CS_PIN, HIGH);
    pinMode(L6474_RESET_PIN, OUTPUT);
    digitalWrite(L6474_RESET_PIN, LOW);
    delay(1);
    digitalWrite(L6474_RESET_PIN, HIGH);
    delay(1);

    SPI.begin();

    // Match the vendored L6474 configuration: 1/16 step, 250 mA TVAL,
    // 3 A overcurrent threshold and the shield's default timing values.
    writeRegister(L6474_ABS_POS_REG, 0, 3);
    writeRegister(0x02, 0, 2);                // EL_POS
    writeRegister(0x03, 0, 3);                // MARK
    writeRegister(L6474_TVAL_REG, 7, 1);     // 250 mA as encoded by the source library
    writeRegister(L6474_T_FAST_REG, 0x34, 1); // 8 us fast decay, 12 us step
    writeRegister(L6474_TON_MIN_REG, 5, 1);
    writeRegister(L6474_TOFF_MIN_REG, 41, 1);
    writeRegister(L6474_OCD_TH_REG, 0x0F, 1);    // 6 A
    writeRegister(L6474_STEP_MODE_REG, 0x8C, 1); // 1/16 step, 1/2 sync
    writeRegister(L6474_ALARM_EN_REG, 0xCF, 1);
    writeRegister(L6474_CONFIG_REG, 0x2C80, 2);
    sendCommand(L6474_DISABLE);
    clearStatus();
    sendCommand(L6474_ENABLE);

    _lastControlTimeUs = micros();
    xTaskCreate(controlTask, "stepper control", 512, this, 2, nullptr);
}

void CustomStepper::setVelocitySetPoint(float velocity)
{
    const float maxVelocity = L6474_MAX_STEP_RATE * _resolution;
    if (velocity > maxVelocity)
    {
        velocity = maxVelocity;
    }
    else if (velocity < -maxVelocity)
    {
        velocity = -maxVelocity;
    }
    taskENTER_CRITICAL();
    _velocitySetpointFromUser = velocity;
    taskEXIT_CRITICAL();
}

inline uint8_t Tval_Current_to_Par(float Tval)
{
    return ((uint8_t)(((Tval - 31.25) / 31.25) + 0.5));
}
void CustomStepper::setCurrentLimit(float currentLimit)
{

    //TODO does not seem to work with serial
    currentLimit = constrain(currentLimit, 31.25f, 4000.0f);
    uint8_t value = static_cast<uint8_t>(
        ((currentLimit - 31.25f) / 31.25f) + 0.5f);

    taskENTER_CRITICAL();
    writeRegister(L6474_TVAL_REG, value, 1);
    taskEXIT_CRITICAL();
}

void CustomStepper::setAcceleration(float accMax)
{
    if (accMax <= 0.0f)
    {
        return;
    }
    taskENTER_CRITICAL();
    _accMax = accMax;
    taskEXIT_CRITICAL();
}

float CustomStepper::getPosition() const
{
    uint32_t rawPosition = readRegister(L6474_ABS_POS_REG, 3) & 0x003FFFFF;
    int32_t stepPosition = (rawPosition & 0x00200000)
                               ? static_cast<int32_t>(rawPosition | 0xFFC00000)
                               : static_cast<int32_t>(rawPosition);
    return stepPosition * _resolution;
}

void CustomStepper::controlTask(void *arg)
{
    auto *stepper = static_cast<CustomStepper *>(arg);
    TickType_t lastWake = xTaskGetTickCount();

    for (;;)
    {
        float setpoint;
        float acceleration;
        taskENTER_CRITICAL();
        setpoint = stepper->_velocitySetpointFromUser;
        acceleration = stepper->_accMax;
        taskEXIT_CRITICAL();

        uint32_t nowUs = micros();
        uint32_t elapsedUs = nowUs - stepper->_lastControlTimeUs;
        stepper->_lastControlTimeUs = nowUs;
        float dt = elapsedUs / 1000000.0f;

        float maxChange = acceleration * dt;
        float velocityError = setpoint - stepper->_velocity;
        if (velocityError > maxChange)
        {
            velocityError = maxChange;
        }
        else if (velocityError < -maxChange)
        {
            velocityError = -maxChange;
        }
        stepper->_velocity += velocityError;

        bool forward = stepper->_velocity >= 0.0f;
        float stepRate = fabsf(stepper->_velocity) / stepper->_resolution;
        if (stepRate < 1.0f)
        {
            analogWrite(stepper->_pinStep, 0);
        }
        else
        {
            if (forward != stepper->_outputForward)
            {
                analogWrite(stepper->_pinStep, 0);
                digitalWrite(stepper->_pinDir, forward ? HIGH : LOW);
                stepper->_outputForward = forward;
            }
            uint32_t frequency = static_cast<uint32_t>(stepRate + 0.5f);
            if (frequency > L6474_MAX_STEP_RATE)
            {
                frequency = L6474_MAX_STEP_RATE;
            }
            if (frequency != stepper->_lastStepRate)
            {
                analogWriteFrequency(frequency);
                analogWrite(stepper->_pinStep, 128);
                stepper->_lastStepRate = frequency;
            }
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}
