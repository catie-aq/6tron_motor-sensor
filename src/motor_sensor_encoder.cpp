/*
 * Copyright (c) 2023, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "motor_sensor/motor_sensor_encoder.h"

namespace sixtron {

MotorSensorEncoder::MotorSensorEncoder(float rate_dt,
        int32_t sensorResolution,
        float motorResolution,
        float motorWheelRadius,
        int sensorDirection):
        _motorResolution(motorResolution),
        _motorWheelRadius(motorWheelRadius),
        _updateRate_dt(rate_dt),
        _sensorResolution(sensorResolution),
        _sensorDirection(sensorDirection)
{
    // Set values
    _wheelPerimeter = (2.0f * float(M_PI) * _motorWheelRadius);
    _tickPerMeters = ((1.0f / (_wheelPerimeter)) * _motorResolution);
}

MotorSensorEncoder::~MotorSensorEncoder() = default;

void MotorSensorEncoder::init()
{

    // Init sensor
    initSensor();

    // Update sensor
    update();

    // Update offset
    _sensorOffset = _sensorTickCount;
}

void MotorSensorEncoder::update()
{

    uint16_t countOld = _sensorRaw;
    uint16_t countNow = 0;

    countNow = getSensorValue();

    int32_t countDelta = (int16_t)(countNow) - (int16_t)(countOld);

    _sensorRaw = countNow;

    // Check if we did an overflow
    if (countDelta >= ((int32_t)_sensorResolution / 2)) {
        _sensorRevol = _sensorRevol - 1;
    } else if (countDelta <= -((int32_t)_sensorResolution / 2)) {
        _sensorRevol = _sensorRevol + 1;
    }

    _sensorTickCountOld = _sensorTickCount;
    _sensorTickCount = -_sensorOffset
            + (_sensorDirection * _sensorRevol * ((int64_t)_sensorResolution))
            + (_sensorDirection * (int64_t)countNow);
}

int64_t MotorSensorEncoder::getTickCount()
{
    return _sensorTickCount;
}

float MotorSensorEncoder::getSpeed()
{
    return ((ticks2Meters(float(_sensorTickCount)) - ticks2Meters(float(_sensorTickCountOld)))
            / _updateRate_dt);
}

float MotorSensorEncoder::getAngle()
{
    // TODO: could this be improve by not using fmod() ?
    return fmod((ticks2Rads(float(_sensorTickCount))), 2.0f * float(M_PI));
}

} // namespace sixtron
