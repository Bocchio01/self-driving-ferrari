/**
 * @file as5600.hpp
 * @brief Header file for the AS5600 magnetic rotary encoder interface.
 *
 * This file contains the declaration of the AS5600 class, which provides methods to interface with the AS5600 sensor.
 * The class allows reading and writing configuration and angle data via I2C, as well as reading the angle from the analog output pin (OUT) if connected.
 *
 * @author Tommaso Bocchietti <tommaso.bocchietti@gmail.com>
 * @date 2026-07-10
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>

/**
 * @brief Class to interface with the AS5600 magnetic rotary encoder.
 *
 * AS5600 exposes a I2C interface to read and write configuration and angle data.
 * Is also possible to read the angle from the analog output pin (OUT) if connected.
 *
 * This class provides exhaustive methods to read and write all the registers of the AS5600, as well as
 * methods to read the angle, compute the angular speed, and set the zero position.
 *
 * The class is designed to be used with the Arduino framework and requires the TwoWire library for I2C communication.
 */
class AS5600
{

public:
    /**
     * @brief Constant representing an unconnected pin.
     */
    static constexpr uint8_t NO_PIN = std::numeric_limits<uint8_t>::max();

    /**
     * @brief Enum class for specifying the type of error that occurred.
     */
    enum class Error : uint8_t
    {
        NONE = 0,
        I2C_READ,
        I2C_WRITE,
        INVALID_PIN
    };

    /**
     * @brief Enum class for specifying the rotation direction.
     */
    enum class Direction : uint8_t
    {
        CLOCKWISE = 0,
        COUNTER_CLOCKWISE
    };

    enum class AngleUnit : uint8_t
    {
        TICK = 0,
        DEGREES,
        RADIANS
    };

    /**
     * @brief Enum class for specifying the unit of angular speed.
     */
    enum class SpeedUnit : uint8_t
    {
        TICK = 0,
        DEGREES_PER_SECOND,
        RADIANS_PER_SECOND,
        RPM
    };

    /**
     * @brief Enum class for specifying the power mode.
     */
    enum class PowerMode : uint8_t
    {
        NOMINAL = 0,
        LOW1,
        LOW2,
        LOW3
    };

    /**
     * @brief Enum class for specifying the hysteresis.
     */
    enum class Hysteresis : uint8_t
    {
        OFF = 0,
        LSB1,
        LSB2,
        LSB3
    };

    /**
     * @brief Enum class for specifying the output mode.
     */
    enum class OutputMode : uint8_t
    {
        ANALOG_100 = 0,
        ANALOG_90,
        PWM
    };

    /**
     * @brief Enum class for specifying the PWM frequency.
     */
    enum class PWMFrequency : uint8_t
    {
        F115 = 0,
        F230,
        F460,
        F920
    };

    /**
     * @brief Enum class for specifying the slow filter.
     */
    enum class SlowFilter : uint8_t
    {
        F16X = 0,
        F8X,
        F4X,
        F2X
    };

    /**
     * @brief Enum class for specifying the fast filter.
     */
    enum class FastFilter : uint8_t
    {
        NONE = 0,
        LSB6,
        LSB7,
        LSB9,
        LSB18,
        LSB21,
        LSB24,
        LSB10
    };

    /**
     * @brief Enum class for specifying the watchdog.
     */
    enum class Watchdog : uint8_t
    {
        OFF = 0,
        ON
    };

    /**
     * @brief Struct representing the configuration of the AS5600 sensor.
     */
    struct Configuration
    {
        PowerMode power_mode;
        Hysteresis hysteresis;
        OutputMode output_mode;
        PWMFrequency pwm_frequency;
        SlowFilter slow_filter;
        FastFilter fast_filter;
        Watchdog watchdog;
    };

    /**
     * @brief Struct representing the status of the AS5600 sensor.
     */
    struct Status
    {
        bool magnetTooClose;
        bool magnetTooFar;
        bool magnetDetected;
    };

    /**
     * @brief Construct an AS5600 object
     *
     * This constructor initializes the AS5600 object and informs the library
     * about the pins used for the DIR and OUT signals, as well as the I2C interface to use.
     *
     * If the OUT pin is connected, it will be used to read the angle directly from the analog output
     * instead of reading it from the ANGLE register via I2C.
     * If the DIR pin is connected, it will be used to set the rotation direction of the sensor,
     * otherwise the direction will be set to CLOCKWISE by default.
     *
     * @param I2C The TwoWire instance to use for I2C communication
     * @param DIR The digital pin connected to the AS5600 DIR. Set to NO_PIN if not used.
     * @param OUT The digital pin connected to the AS5600 OUT. Set to NO_PIN if not used.
     */
    explicit AS5600(TwoWire *I2C = &Wire, uint8_t DIR = NO_PIN, uint8_t OUT = NO_PIN);

    /**
     * @brief Initialize the AS5600 sensor
     *
     * This function initializes the AS5600 sensor by setting the pin modes for
     * the DIR and OUT pins, if they are connected.
     * In case of I2C communication, the provided TwoWire instance must be initialized
     * before calling this function.
     *
     * @return True if the initialization was successful, false otherwise.
     */
    bool begin();

    /**
     * @brief Update the internal state of the AS5600 object
     *
     * This function reads the current angle from the sensor and updates all
     * internal state variables, including angles and speed.
     * It should be called periodically or before requesting any measurements
     * to ensure that the data is up-to-date.
     */
    void update();

    /**
     * @brief Read the raw angle from the sensor's hardware registers.
     *
     * This function requires that I2C communication is available as it will read
     * the RAW_ANGLE register directly.
     * If no I2C interface is provided or an I2C error occurs, the last known angle
     * is returned.
     *
     * @return The raw angle value in the range [0, 4095].
     */
    uint16_t readRawAngle() { return readHardwareAngle(Register::Address::RAW_ANGLE); }

    /**
     * @brief Read the angle from the sensor.
     *
     * If the OUT pin is connected, the angle is read from the analog output.
     * Otherwise, the ANGLE register is read via I2C.
     * If an I2C error occurs, the last known angle is returned.
     *
     * This function does not apply the software zero offset and does not update
     * the internal state of the AS5600 object. Use update() and getAngle() to
     * get the current angle with the software zero offset applied.
     *
     * @return The angle value in the range [0, 4095].
     */
    uint16_t readAngle() { return readHardwareAngle(Register::Address::ANGLE); }

    /**
     * @brief Returns the current angle with the software zero offset applied.
     *
     * This function returns the angle value in the range [0, 4095], taking into account
     * the software zero offset set by setZero(). The angle is updated by calling update()
     * before calling this function to ensure that the returned value is up-to-date.
     *
     * @param unit The unit in which to return the angle. Default is TICK.
     * @return The angle value in the range [0, FULL_SCALE] with the software zero offset applied.
     */
    float getAngle(AngleUnit unit = AngleUnit::TICK) const { return tickToAngleUnit(angle_, unit); }

    /**
     * @brief Returns the cumulative angle with the software zero offset applied.
     *
     * This function returns the cumulative angle value, which can be negative or positive,
     * depending on the number of revolutions made by the sensor. The cumulative angle is updated by
     * calling update() before calling this function to ensure that the returned value is up-to-date.
     *
     * @param unit The unit in which to return the cumulative angle. Default is TICK.
     * @return The cumulative angle value
     */
    float getCumulativeAngle(AngleUnit unit = AngleUnit::TICK) const { return tickToAngleUnit(cumulative_angle_, unit); }

    /**
     * @brief Returns the number of revolutions made by the sensor.
     *
     * This function calculates the number of complete revolutions based on the cumulative angle.
     * The number of revolutions is updated by calling update() before calling this function to ensure
     * that the returned value is up-to-date.
     *
     * @return The number of revolutions made by the sensor.
     */
    float getRevolutions() const { return static_cast<float>(cumulative_angle_) / static_cast<float>(RESOLUTION); }

    /**
     * @brief Returns the current angular speed in the specified unit.
     *
     * This function returns the angular speed calculated based on the change in angle over time.
     * The speed is updated by calling update() before calling this function to ensure that the
     * returned value is up-to-date.
     *
     * @param unit The unit in which to return the angular speed. Default is RADIANS_PER_SECOND.
     * @return The angular speed in the specified unit. If the speed cannot be calculated, NAN is returned.
     */
    float getAngularSpeed(SpeedUnit unit = SpeedUnit::RADIANS_PER_SECOND) { return tickToSpeedUnit(current_speed_, unit); }

    /**
     * @brief Set the current position as the zero position.
     *
     * Call this function to set the current position of the sensor as the zero position.
     * This will update the software zero offset, and subsequent calls to getAngle() and getCumulativeAngle()
     * will return values relative to this new zero position.
     */
    void setZero();

    /**
     * @brief Set the direction of the sensor.
     *
     * The sensor can be informed about the direction of rotation.
     * This is useful when the sensor is mounted in a way that the rotation direction is reversed
     * compared to the default clockwise direction.
     *
     * @param direction The direction to set for the sensor.
     */
    void setDirection(Direction direction);

    /**
     * @brief Get the direction of the sensor.
     * @return The direction of the sensor.
     */
    Direction getDirection() const { return direction_; }

    /**
     * @brief Set the software offset.
     *
     * This function sets the software offset in degrees and overrides the zero position set by setZero().
     * The offset is applied to the angle returned by getAngle() and getCumulativeAngle().
     *
     * @param degrees The offset in degrees.
     */
    void setOffset(float degrees);

    /**
     * @brief Get the software offset.
     * @return The software offset in degrees.
     */
    float getOffset() const { return tickToAngleUnit(raw_offset_, AngleUnit::DEGREES); }

    /**
     * @brief Set the ADC resolution for reading the angle from the OUT pin.
     *
     * This function sets the ADC resolution used when reading the angle from the OUT pin.
     * The default resolution is 10 bits. Valid values are 8, 10, 12, and 14 bits.
     * If the OUT pin is not connected, this setting has no effect.
     *
     * @param bits The ADC resolution in bits (8, 10, 12, or 14).
     */
    void setADCResolution(uint8_t bits) { adc_resolution_ = bits; }

    /**
     * @brief Get the ADC resolution for reading the angle from the OUT pin.
     * @return The ADC resolution in bits.
     */
    uint8_t getADCResolution() const { return adc_resolution_; }

    /**
     * @brief Clear the current error state.
     *
     * This function clears any error state that may have occurred during I2C communication or other operations.
     * After calling this function, getError() will return Error::NONE.
     */
    void clearError() { error_ = Error::NONE; }

    /**
     * @brief Get the current error state.
     * @return The current error state as an Error enum value.
     */
    Error getError() const { return error_; }

    // =======================================================
    // Register Accessors
    // =======================================================

    uint8_t getZMCO() const { return readRegister(Register::zero_magnet_count); }
    uint16_t getZPosition() const { return readRegister(Register::z_position); }
    uint16_t getMPosition() const { return readRegister(Register::m_position); }
    uint16_t getMaxAngle() const { return readRegister(Register::max_angle); }
    uint8_t getAGC() const { return readRegister(Register::agc); }
    uint16_t getMagnitude() const { return readRegister(Register::magnitude); }
    Configuration getConfiguration() const;
    Status getStatus() const;

    bool setZPosition(uint16_t value) { return writeRegister(Register::z_position, value, (uint16_t)(RESOLUTION - 1)); }
    bool setMPosition(uint16_t value) { return writeRegister(Register::m_position, value, (uint16_t)(RESOLUTION - 1)); }
    bool setMaxAngle(uint16_t value) { return writeRegister(Register::max_angle, value, (uint16_t)(RESOLUTION - 1)); }
    bool setConfiguration(const Configuration &config);

protected:
    static constexpr uint16_t RESOLUTION = 4096;

    TwoWire *I2C_ = nullptr;
    const uint8_t ADDR_ = 0x36; // Const I2C address of the AS5600 sensor
    const uint8_t OUT_ = NO_PIN;
    const uint8_t DIR_ = NO_PIN;

    Direction direction_ = Direction::CLOCKWISE;
    mutable Error error_ = Error::NONE;

    uint8_t adc_resolution_ = 10;
    uint16_t raw_offset_ = 0;
    uint16_t angle_ = 0;
    int32_t cumulative_angle_ = 0;
    uint32_t angle_timestamp_ = 0;
    float current_speed_ = 0.0f;

    /**
     * @brief Convert raw ticks to a specified angle unit
     * @param angle The raw tick value
     * @param unit The desired angle unit
     * @return The corresponding value in the specified unit
     */
    template <typename T>
    static inline constexpr float tickToAngleUnit(T angle, AngleUnit unit)
    {
        float f_angle = static_cast<float>(angle);

        switch (unit)
        {
        case AngleUnit::TICK:
            return f_angle;
        case AngleUnit::DEGREES:
            return f_angle * (360.0f / static_cast<float>(RESOLUTION));
        case AngleUnit::RADIANS:
            return f_angle * (2.0f * PI / static_cast<float>(RESOLUTION));
        default:
            return NAN;
        }
    }

    /**
     * @brief Convert raw speed to a specified speed unit
     * @param speed The raw speed value
     * @param unit The desired speed unit
     * @return The corresponding value in the specified unit
     */
    template <typename T>
    static inline constexpr float tickToSpeedUnit(T speed, SpeedUnit unit)
    {
        float f_speed = static_cast<float>(speed);

        switch (unit)
        {
        case SpeedUnit::TICK:
            return f_speed;
        case SpeedUnit::DEGREES_PER_SECOND:
            return f_speed * (360.0f / static_cast<float>(RESOLUTION));
        case SpeedUnit::RADIANS_PER_SECOND:
            return f_speed * (2.0f * PI / static_cast<float>(RESOLUTION));
        case SpeedUnit::RPM:
            return f_speed * (60.0f / static_cast<float>(RESOLUTION));
        default:
            return NAN;
        }
    }

private:
    struct Register
    {

        /**
         * @brief Enum representing the register addresses of the AS5600
         */
        enum class Address : uint8_t
        {
            ZMCO = 0x00,
            ZPOS = 0x01,
            MPOS = 0x03,
            MANG = 0x05,
            CONF = 0x07,
            RAW_ANGLE = 0x0C,
            ANGLE = 0x0E,
            STATUS = 0x0B,
            AGC = 0x1A,
            MAGNITUDE = 0x1B,
            BURN = 0xFF,
        };

        /**
         * @brief Structure representing a register entry with its address and mask
         * @tparam T The type of the register value (uint8_t or uint16_t)
         */
        template <typename T>
        struct Field
        {
            Address register_address;
            T mask;

            constexpr uint8_t shift() const
            {
                uint8_t value = 0;
                T m = mask;

                while (m && ((m & 1) == 0))
                {
                    m >>= 1;
                    value++;
                }

                return value;
            }
        };

        // Configuration Registers
        static constexpr Field<uint8_t> zero_magnet_count{Address::ZMCO, 0x03};
        static constexpr Field<uint16_t> z_position{Address::ZPOS, 0x0FFF};
        static constexpr Field<uint16_t> m_position{Address::MPOS, 0x0FFF};
        static constexpr Field<uint16_t> max_angle{Address::MANG, 0x0FFF};
        static constexpr Field<uint16_t> configuration{Address::CONF, 0x7FFF};
        static constexpr Field<uint16_t> power_mode{Address::CONF, 0x0003};
        static constexpr Field<uint16_t> hysteresis{Address::CONF, 0x000C};
        static constexpr Field<uint16_t> output_mode{Address::CONF, 0x0030};
        static constexpr Field<uint16_t> pwm_frequency{Address::CONF, 0x00C0};
        static constexpr Field<uint16_t> slow_filter{Address::CONF, 0x0300};
        static constexpr Field<uint16_t> fast_filter{Address::CONF, 0x1C00};
        static constexpr Field<uint16_t> watchdog{Address::CONF, 0x2000};

        // Output Registers
        static constexpr Field<uint16_t> raw_angle{Address::RAW_ANGLE, 0x0FFF};
        static constexpr Field<uint16_t> angle{Address::ANGLE, 0x0FFF};

        // Status Registers
        static constexpr Field<uint8_t> status{Address::STATUS, 0xFF};
        static constexpr Field<uint8_t> magnet_too_close{Address::STATUS, 0x08};
        static constexpr Field<uint8_t> magnet_too_far{Address::STATUS, 0x10};
        static constexpr Field<uint8_t> magnet_detected{Address::STATUS, 0x20};
        static constexpr Field<uint8_t> agc{Address::AGC, 0xFF};
        static constexpr Field<uint16_t> magnitude{Address::MAGNITUDE, 0x0FFF};
    };

    /**
     * @brief Read the angle from the hardware.
     *
     * Depending on the requested register, two behaviors are possible:
     * - If the OUT pin is connected and the requested register is ANGLE, the angle
     * is read from the analog output pin (OUT) using analogRead().
     * - If the OUT pin is not connected or the requested register is RAW_ANGLE, the
     * angle is read from the corresponding register via I2C.
     *
     * If an I2C error occurs, the last known angle is returned.
     *
     * @param address The register address to read the angle from (ANGLE or RAW_ANGLE).
     * @return The angle value in the range [0, 4095].
     */
    uint16_t readHardwareAngle(Register::Address address);

    /**
     * @brief Read a single byte from the specified register
     * @param reg The register address to read from
     * @param value The variable to store the read value
     * @return True if the read was successful, false otherwise
     */
    bool read(Register::Address reg, uint8_t &value) const;

    /**
     * @brief Read a 16-bit value from the specified register
     * @param reg The register address to read from
     * @param value The variable to store the read value
     * @return True if the read was successful, false otherwise
     */
    bool read(Register::Address reg, uint16_t &value) const;

    /**
     * @brief Write a single byte to the specified register
     * @param reg The register address to write to
     * @param value The value to write
     * @return True if the write was successful, false otherwise
     */
    bool write(Register::Address reg, uint8_t value) const;

    /**
     * @brief Write a 16-bit value to the specified register
     * @param reg The register address to write to
     * @param value The value to write
     * @return True if the write was successful, false otherwise
     */
    bool write(Register::Address reg, uint16_t value) const;

    /**
     * @brief Read a value from the specified register field
     * @param field The register field to read from
     * @return The read value
     */
    template <typename T>
    T readRegister(const Register::Field<T> &field) const
    {
        T value = 0;

        if (!read(field.register_address, value))
        {
            return static_cast<T>(0);
        }

        return static_cast<T>((value & field.mask) >> field.shift());
    }

    /**
     * @brief Write a value to the specified register field
     * @param field The register field to write to
     * @param value The value to write
     * @param limit The maximum allowed value
     * @return True if the write was successful, false otherwise
     */
    template <typename T>
    bool writeRegister(const Register::Field<T> &field, T value, T limit = std::numeric_limits<T>::max())
    {
        T current = 0;

        if (value > limit)
            return false;

        if (!read(field.register_address, current))
            return false;

        current &= ~field.mask;
        current |= (value << field.shift()) & field.mask;

        return write(field.register_address, current);
    }
};