#pragma once

#include <Wire.h>
#include "sensors/sensor.hpp"

namespace sensors
{

    struct InertiaMeasurementUnitOutput
    {
        float quaternion_x;
        float quaternion_y;
        float quaternion_z;
        float quaternion_w;
        float gyro_x;
        float gyro_y;
        float gyro_z;
    };

    class InertiaMeasurementUnit : public interfaces::Sensor<InertiaMeasurementUnitOutput>, public AS5600
    {

    public:
        InertiaMeasurementUnit(TwoWire *i2c)
        {
        }

        InertiaMeasurementUnitOutput data() override
        {
            return {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f};
        }
    };

}