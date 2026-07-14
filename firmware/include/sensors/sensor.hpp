#pragma once

namespace sensors::interfaces
{
    template <typename T>
    class Sensor
    {
    public:
        virtual ~Sensor() = default;
        virtual T data() = 0;
    };
}