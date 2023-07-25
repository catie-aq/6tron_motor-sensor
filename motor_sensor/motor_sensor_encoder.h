/*
 * Copyright (c) 2023, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CATIE_SIXTRON_MOTORSENSOR_ENCODER_H
#define CATIE_SIXTRON_MOTORSENSOR_ENCODER_H

#include "mbed.h"
#include "motor_sensor.h"

namespace sixtron {

#define DIR_NORMAL (+1)
#define DIR_INVERTED (-1)

class MotorSensorEncoder: public MotorSensor {
public:
    MotorSensorEncoder(float rate_dt,
            int32_t sensorResolution,
            float motorResolution,
            float motorWheelRadius,
            int encDirection = DIR_NORMAL);

    ~MotorSensorEncoder();

    void init() override;

    void update() override;

    int64_t getTickCount() override; // raw tick count.

    float getSpeed() override; // in [m/s], with gearbox and wheel.

    float getAngle() override; // in [rad].

protected:
    // THESE FUNCTION MUST BE REDEFINED REGARDING THE APPLICATION
    virtual void initSensor() = 0;
    virtual uint16_t getSensorValue() = 0;

    // motor data
    float _motorResolution, _motorWheelRadius;
    float _wheelPerimeter, _tickPerMeters;

    // sensor data
    float _updateRate_dt = 0.0f;
    int64_t _sensorResolution = 0;
    int64_t _sensorDirection = 0;

private:
    inline float ticks2Meters(float ticks) const
    {
        return (ticks * (1.0f / _tickPerMeters));
    }

    inline float ticks2Rads(float ticks) const
    {
        return ((ticks * float(M_PI)) / (_motorResolution / 2.0f));
    }

    uint16_t _sensorRaw = 0;
    int32_t _sensorRevol = 0;
    int64_t _sensorTickCount = 0;
    int64_t _sensorTickCountOld = 0;
    int64_t _sensorOffset = 0;
};

} // namespace sixtron

#endif // CATIE_SIXTRON_MOTORSENSOR_ENCODER_H
