/*
 * Copyright (c) 2023, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "motor_sensor/motor_sensor_mbed_AS5047p.h"

namespace sixtron {

    MotorSensorMbedAS5047P::MotorSensorMbedAS5047P(SPI *spiAS5047p,
                                                   PinName sensor_spi_select,
                                                   float rateHz,
                                                   int32_t sensorResolution,
                                                   float motorResolution,
                                                   float motorWheelRadius,
                                                   int sensorDirection) :
            _motorResolution(motorResolution),
            _motorWheelRadius(motorWheelRadius),
            _spiAS5047p(spiAS5047p),
            _as5047p_spi_cs(sensor_spi_select),
            _updateRateHz(rateHz),
            _sensorResolution(sensorResolution),
            _sensorDirection(sensorDirection) {

        // configure SPI
        _spiAS5047p->format(8, 1);
        _spiAS5047p->frequency(1000000);

        // configure GPIO SPI selection
        _as5047p_spi_cs.write(1);

        // Set values
        _wheelPerimeter = (2.0f * float(M_PI) * _motorWheelRadius);
        _tickPerMeters = ((1.0f / (_wheelPerimeter)) * _motorResolution);

    }

    MotorSensorMbedAS5047P::~MotorSensorMbedAS5047P() = default;

    void MotorSensorMbedAS5047P::init() {

        // Update sensor
        update();

        // Update offset
        _sensorOffset = _sensorTickCount;

    }

    void MotorSensorMbedAS5047P::update() {

        uint16_t countOld = _sensorRaw;
        uint16_t countNow = 0;

        countNow = getSensorValue();

        int32_t countDelta = (int16_t) (countNow) - (int16_t) (countOld);

        _sensorRaw = countNow;

        // Check if we did an overflow
        if (countDelta >= ((int32_t) _sensorResolution / 2)) {
            _sensorRevol = _sensorRevol - 1;
        } else if (countDelta <= -((int32_t) _sensorResolution / 2)) {
            _sensorRevol = _sensorRevol + 1;
        }

        _sensorTickCountOld = _sensorTickCount;
        _sensorTickCount = -_sensorOffset +
                           (_sensorDirection * _sensorRevol * ((int64_t) _sensorResolution)) +
                           (_sensorDirection * (int64_t) countNow);

    }

    uint16_t MotorSensorMbedAS5047P::getSensorValue() {
        _as5047p_spi_cs.write(0);
        uint8_t receivedDataH = (0x3F & _spiAS5047p->write(0xFF)); //Get the first part (8bits)
        uint8_t receivedDataL = _spiAS5047p->write(0xFF); //Get the second part (8bits)
        _as5047p_spi_cs.write(1);
        return (uint16_t) ((receivedDataH << 8) | (receivedDataL & 0xff)); //Combine the two parts to get a 16bits
    }

    int64_t MotorSensorMbedAS5047P::getTickCount() {
        return _sensorTickCount;
    }

    float MotorSensorMbedAS5047P::getSpeed() {

        return ((ticks2Meters(float(_sensorTickCount)) - ticks2Meters(float(_sensorTickCountOld))) / _updateRateHz);

    }

    float MotorSensorMbedAS5047P::getAngle() {

        // TODO: could this be improve by not using fmod() ?
        return fmod((ticks2Rads(float(_sensorTickCount))), 2.0f * float(M_PI));
    }


} // namespace sixtron
