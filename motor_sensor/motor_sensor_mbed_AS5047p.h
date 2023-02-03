/*
 * Copyright (c) 2023, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CATIE_SIXTRON_MOTORSENSOR_MBED_AS147P_H
#define CATIE_SIXTRON_MOTORSENSOR_MBED_AS147P_H

#include "mbed.h"
#include "motor_sensor.h"

namespace sixtron {

#define DIR_NORMAL (+1)
#define DIR_INVERTED (-1)

class MotorSensorMbedAS5047P: public MotorSensor {
public:
    MotorSensorMbedAS5047P(SPI *spiAS5047p,
            PinName sensor_spi_select,
            float rateHz,
            int32_t sensorResolution,
            float motorResolution,
            float motorWheelRadius,
            int encDirection = DIR_NORMAL);

    ~MotorSensorMbedAS5047P();

    void init() override;

    void update() override;

    int64_t getTickCount() override; // raw tick count.

    float getSpeed() override; // in [m/s], with gearbox and wheel.

    float getAngle() override; // in [rad].

private:
    inline float ticks2Meters(float ticks) const
    {
        return (ticks * (1.0f / _tickPerMeters));
    }

    inline float ticks2Rads(float ticks) const
    {
        return ((ticks * float(M_PI)) / (_motorResolution / 2.0f));
    }

    float _motorResolution, _motorWheelRadius;
    float _wheelPerimeter, _tickPerMeters;

    uint16_t getSensorValue();

    SPI *_spiAS5047p;
    DigitalOut _as5047p_spi_cs;

    // sensor data
    float _updateRateHz = 0.0f;
    int64_t _sensorResolution = 0;
    int64_t _sensorDirection = 0;

    uint16_t _sensorRaw = 0;
    int32_t _sensorRevol = 0;
    int64_t _sensorTickCount = 0;
    int64_t _sensorTickCountOld = 0;
    int64_t _sensorOffset = 0;
};

} // namespace sixtron

#endif // CATIE_SIXTRON_MOTORSENSOR_MBED_AS147P_H
