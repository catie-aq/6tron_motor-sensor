/*
 * Copyright (c) 2023, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CATIE_SIXTRON_MOTORSENSOR_H
#define CATIE_SIXTRON_MOTORSENSOR_H

#include <stdint.h>
#include <math.h>

namespace sixtron {

    class MotorSensor {

    public:
        virtual void init() = 0;

        virtual void update() = 0;

        virtual int64_t getTickCount() = 0; // raw count, depending on the sensor

        virtual float getSpeed() = 0; // in [m/s], with gearbox and wheel ? or just raw ?

        virtual float getAngle() = 0; // in [rad]

    };

} // namespace sixtron

#endif // CATIE_SIXTRON_MOTORSENSOR_H
