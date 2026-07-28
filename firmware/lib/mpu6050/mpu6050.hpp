/**
 * @file mpu6050.hpp
 * @brief Header file for the MPU6050 6-axis accelerometer/gyroscope interface.
 *
 * This file contains the declaration of the MPU6050 class, which provides a self-contained
 * interface to the InvenSense MPU-6050 IMU over I2C. No external dependencies beyond Arduino/Wire
 * are required.
 *
 * The class exposes the commonly used subset of the device (power management, full scale
 * range configuration, sample rate / low pass filter configuration, raw and converted motion
 * data, gyroscope offset calibration, FIFO buffer and data-ready interrupt configuration).
 * Auxiliary I2C master (slave passthrough), DMP and motion/free-fall/zero-motion detection
 * registers are intentionally not implemented — see the note at the end of the .cpp file.
 *
 * @author Tommaso Bocchietti <tommaso.bocchietti@gmail.com>
 * @date 2026-07-28
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <limits>

/**
 * @brief Class to interface with the MPU6050 6-axis accelerometer/gyroscope.
 *
 * The MPU6050 exposes a I2C interface to read/write configuration registers and to read
 * raw accelerometer, gyroscope and temperature data.
 *
 * This class provides methods to configure the sensor (full scale ranges, sample rate,
 * digital low pass filter, sleep/power modes, data-ready interrupt, FIFO), to read raw and
 * unit-converted motion data, and to apply a software gyroscope offset calibration.
 *
 * The class is designed to be used with the Arduino framework and requires the TwoWire
 * library for I2C communication.
 */
class MPU6050
{

public:
    /**
     * @brief Default I2C address of the MPU6050 (AD0 pin tied low).
     */
    static constexpr uint8_t DEFAULT_ADDRESS = 0x68;

    /**
     * @brief Alternate I2C address of the MPU6050 (AD0 pin tied high).
     */
    static constexpr uint8_t ALTERNATE_ADDRESS = 0x69;

    /**
     * @brief Enum class for specifying the type of error that occurred.
     */
    enum class Error : uint8_t
    {
        NONE = 0,
        I2C_READ,
        I2C_WRITE
    };

    /**
     * @brief Enum class for specifying the device clock source.
     */
    enum class ClockSource : uint8_t
    {
        INTERNAL_8MHZ = 0,
        PLL_X_GYRO = 1,
        PLL_Y_GYRO = 2,
        PLL_Z_GYRO = 3,
        PLL_EXTERNAL_32KHZ = 4,
        PLL_EXTERNAL_19MHZ = 5,
        STOPPED = 7
    };

    /**
     * @brief Enum class for specifying the gyroscope full scale range.
     */
    enum class GyroRange : uint8_t
    {
        DPS_250 = 0,
        DPS_500 = 1,
        DPS_1000 = 2,
        DPS_2000 = 3
    };

    /**
     * @brief Enum class for specifying the accelerometer full scale range.
     */
    enum class AccelRange : uint8_t
    {
        G_2 = 0,
        G_4 = 1,
        G_8 = 2,
        G_16 = 3
    };

    /**
     * @brief Enum class for specifying the digital low pass filter bandwidth.
     *
     * Selecting anything other than HZ_260 also drops the internal sampling rate
     * used to derive the Sample Rate (see setSampleRateDivider()) from 8kHz to 1kHz.
     */
    enum class DLPFBandwidth : uint8_t
    {
        HZ_260 = 0,
        HZ_184 = 1,
        HZ_94 = 2,
        HZ_44 = 3,
        HZ_21 = 4,
        HZ_10 = 5,
        HZ_5 = 6
    };

    /**
     * @brief Enum class for specifying the wake-up frequency in accelerometer-only low power mode.
     */
    enum class LowPowerWakeFrequency : uint8_t
    {
        HZ_1_25 = 0,
        HZ_5 = 1,
        HZ_20 = 2,
        HZ_40 = 3
    };

    /**
     * @brief Enum class for specifying the unit of acceleration.
     */
    enum class AccelUnit : uint8_t
    {
        G = 0,
        MPS2
    };

    /**
     * @brief Enum class for specifying the unit of angular rate.
     */
    enum class GyroUnit : uint8_t
    {
        DPS = 0,
        RPS
    };

    /**
     * @brief Enum class for specifying the unit of temperature.
     */
    enum class TempUnit : uint8_t
    {
        CELSIUS = 0,
        FAHRENHEIT
    };

    /**
     * @brief Simple 3-axis container, used both for raw ticks and for converted values.
     */
    template <typename T>
    struct Axis3
    {
        T x;
        T y;
        T z;
    };

    /**
     * @brief Construct a MPU6050 object.
     *
     * @param I2C The TwoWire instance to use for I2C communication.
     * @param address The I2C address of the device (DEFAULT_ADDRESS or ALTERNATE_ADDRESS
     *                depending on the state of the AD0 pin).
     */
    explicit MPU6050(TwoWire *I2C = &Wire, uint8_t address = DEFAULT_ADDRESS);

    /**
     * @brief Initialize the MPU6050 sensor.
     *
     * Wakes the device up from sleep, sets the clock source to the X gyroscope PLL
     * (recommended over the internal oscillator for stability), and applies the requested
     * full scale ranges.
     *
     * The provided TwoWire instance must be initialized (Wire.begin()) before calling this.
     *
     * @param accel_range Accelerometer full scale range to configure.
     * @param gyro_range Gyroscope full scale range to configure.
     * @return True if the device was initialized and testConnection() succeeds, false otherwise.
     */
    bool begin(AccelRange accel_range = AccelRange::G_2, GyroRange gyro_range = GyroRange::DPS_250);

    /**
     * @brief Verify the I2C connection by checking the WHO_AM_I register.
     * @return True if the device responds with the expected device ID, false otherwise.
     */
    bool testConnection() const;

    /**
     * @brief Trigger a full device reset (registers restored to default values).
     *
     * The bit is self-clearing on the device; a short settling delay is applied after
     * triggering the reset.
     */
    void reset();

    // =======================================================
    // Power Management
    // =======================================================

    void setSleepEnabled(bool enabled);
    bool getSleepEnabled() const;

    void setClockSource(ClockSource source);
    ClockSource getClockSource() const;

    void setCycleEnabled(bool enabled);
    bool getCycleEnabled() const;

    void setTemperatureSensorEnabled(bool enabled);
    bool getTemperatureSensorEnabled() const;

    void setLowPowerWakeFrequency(LowPowerWakeFrequency frequency);
    LowPowerWakeFrequency getLowPowerWakeFrequency() const;

    // =======================================================
    // Full Scale Range Configuration
    // =======================================================

    void setGyroRange(GyroRange range);
    GyroRange getGyroRange() const { return gyro_range_; }

    void setAccelRange(AccelRange range);
    AccelRange getAccelRange() const { return accel_range_; }

    // =======================================================
    // Sample Rate / Filtering
    // =======================================================

    /**
     * @brief Set the gyroscope output rate divider.
     *
     * Sample Rate = Gyroscope Output Rate / (1 + divider), where the Gyroscope Output Rate
     * is 8kHz if the DLPF is set to HZ_260, and 1kHz for any other DLPF setting.
     */
    void setSampleRateDivider(uint8_t divider);
    uint8_t getSampleRateDivider() const;

    void setDLPFBandwidth(DLPFBandwidth bandwidth);
    DLPFBandwidth getDLPFBandwidth() const;

    // =======================================================
    // Data-Ready Interrupt
    // =======================================================

    /**
     * @brief Configure the behavior of the INT pin.
     * @param active_low If true, the pin is active-low instead of active-high.
     * @param open_drain If true, the pin is configured as open-drain instead of push-pull.
     * @param latch If true, the interrupt is latched until cleared instead of a 50us pulse.
     * @param clear_on_any_read If true, any register read clears the latched interrupt,
     *                          instead of only a read of INT_STATUS.
     */
    void setInterruptPinConfig(bool active_low, bool open_drain, bool latch, bool clear_on_any_read);

    void setDataReadyInterruptEnabled(bool enabled);
    bool getDataReadyInterruptEnabled() const;

    /**
     * @brief Get the Data Ready interrupt status. Clears the flag on the device once read.
     */
    bool getDataReadyInterruptStatus() const;

    // =======================================================
    // Sensor Measurement
    // =======================================================

    /**
     * @brief Read the raw accelerometer ticks directly from the device.
     */
    Axis3<int16_t> readRawAcceleration() const;

    /**
     * @brief Read the raw gyroscope ticks directly from the device.
     */
    Axis3<int16_t> readRawGyroscope() const;

    /**
     * @brief Read the raw temperature ticks directly from the device.
     */
    int16_t readRawTemperature() const;

    /**
     * @brief Read the acceleration directly from the device, converted to the requested unit.
     */
    Axis3<float> readAcceleration(AccelUnit unit = AccelUnit::G) const;

    /**
     * @brief Read the angular rate directly from the device, converted to the requested unit.
     */
    Axis3<float> readGyroscope(GyroUnit unit = GyroUnit::DPS) const;

    /**
     * @brief Read the die temperature directly from the device, converted to the requested unit.
     */
    float readTemperature(TempUnit unit = TempUnit::CELSIUS) const;

    /**
     * @brief Update the internal cached state of the MPU6050 object.
     *
     * Performs a single burst read of accelerometer, temperature and gyroscope registers
     * (more efficient than three separate reads) and caches the converted values
     * (g, degrees/s, degrees C) along with a timestamp. Should be called periodically before
     * requesting cached measurements.
     */
    void update();

    Axis3<float> getAcceleration() const { return acceleration_; }
    Axis3<float> getGyroscope() const { return gyroscope_; }
    float getTemperature() const { return temperature_; }

    /**
     * @brief Get the timestamp (in microseconds, from micros()) of the last update() call.
     */
    uint32_t getTimestamp() const { return timestamp_; }

    // =======================================================
    // Offset Calibration
    // =======================================================

    /**
     * @brief Set the hardware gyroscope offset (applied by the device before output).
     *
     * The gyroscope offset registers are documented and consistent across
     * MPU6050/MPU6500/MPU9150/MPU9250 variants, so the correction is applied on-chip.
     * Persists only until the next power-cycle/reset — reapply it in setup() after
     * calibrating once.
     */
    void setGyroOffset(Axis3<int16_t> offset);

    /**
     * @brief Get the currently configured hardware gyroscope offset.
     */
    Axis3<int16_t> getGyroOffset() const;

    /**
     * @brief Set a software accelerometer offset.
     *
     * Subtracted from every raw accelerometer reading (readRawAcceleration(),
     * readAcceleration(), and the accelerometer values cached by update()/getAcceleration()).
     * Unlike setGyroOffset(), this is not written to the device: the MPU6050's accelerometer
     * hardware offset registers are undocumented and their address varies by chip variant
     * (0x06 on MPU6050/MPU9150 vs 0x77 on MPU6500/MPU9250), which would require the same
     * runtime WHO_AM_I heuristic the original I2Cdevlib code relies on. Applying the
     * correction in software sidesteps that entirely, at the cost of it living only in RAM —
     * it must be reapplied (e.g. hardcoded from a prior calibration run) after every power-up.
     */
    void setAccelOffset(Axis3<int16_t> offset);

    /**
     * @brief Get the currently configured software accelerometer offset.
     */
    Axis3<int16_t> getAccelOffset() const { return accel_offset_; }

    // =======================================================
    // FIFO Buffer
    // =======================================================

    void setFIFOBufferEnabled(bool enabled);
    bool getFIFOBufferEnabled() const;

    /**
     * @brief Reset the FIFO buffer. The bit is self-clearing on the device.
     */
    void resetFIFOBuffer();

    /**
     * @brief Select which sensor outputs are written into the FIFO buffer.
     */
    void setFIFOEnabledSources(bool temperature, bool accel, bool gyro_x, bool gyro_y, bool gyro_z);

    /**
     * @brief Get the current number of bytes stored in the FIFO buffer.
     */
    uint16_t getFIFOCount() const;

    /**
     * @brief Read raw bytes out of the FIFO buffer.
     * @param data Destination buffer, must be at least `length` bytes.
     * @param length Number of bytes to read.
     */
    void readFIFOBytes(uint8_t *data, uint8_t length) const;

    // =======================================================
    // Identification / Error Handling
    // =======================================================

    /**
     * @brief Get the 6-bit device ID (should read 0x34 for the MPU6050).
     */
    uint8_t getDeviceID() const;

    /**
     * @brief Read an arbitrary register directly. Escape hatch for registers not otherwise
     *        exposed by this class (e.g. undocumented registers, DMP, auxiliary I2C master).
     */
    uint8_t readRegister(uint8_t address) const;

    /**
     * @brief Write an arbitrary register directly. Escape hatch for registers not otherwise
     *        exposed by this class (e.g. undocumented registers, DMP, auxiliary I2C master).
     */
    bool writeRegister(uint8_t address, uint8_t value) const;

    void clearError() { error_ = Error::NONE; }
    Error getError() const { return error_; }

private:
    /**
     * @brief 6-bit device ID expected to be read from the WHO_AM_I register.
     */
    static constexpr uint8_t WHO_AM_I_VALUE = 0x34;

    TwoWire *I2C_ = nullptr;
    uint8_t address_ = DEFAULT_ADDRESS;
    mutable Error error_ = Error::NONE;

    AccelRange accel_range_ = AccelRange::G_2;
    GyroRange gyro_range_ = GyroRange::DPS_250;

    Axis3<float> acceleration_{0.0f, 0.0f, 0.0f};
    Axis3<float> gyroscope_{0.0f, 0.0f, 0.0f};
    float temperature_ = 0.0f;
    uint32_t timestamp_ = 0;

    Axis3<int16_t> accel_offset_{0, 0, 0};

    /**
     * @brief Acceleration resolution in g/LSB for the currently configured accel_range_.
     */
    float accelResolution() const;

    /**
     * @brief Angular rate resolution in degrees/s/LSB for the currently configured gyro_range_.
     */
    float gyroResolution() const;

    // =======================================================
    // Register Map
    // =======================================================

    struct Register
    {
        /**
         * @brief Enum representing the register addresses used by this library.
         */
        enum class Address : uint8_t
        {
            XG_OFFS_USRH = 0x13,
            XG_OFFS_USRL = 0x14,
            YG_OFFS_USRH = 0x15,
            YG_OFFS_USRL = 0x16,
            ZG_OFFS_USRH = 0x17,
            ZG_OFFS_USRL = 0x18,
            SMPLRT_DIV = 0x19,
            CONFIG = 0x1A,
            GYRO_CONFIG = 0x1B,
            ACCEL_CONFIG = 0x1C,
            FIFO_EN = 0x23,
            INT_PIN_CFG = 0x37,
            INT_ENABLE = 0x38,
            INT_STATUS = 0x3A,
            ACCEL_XOUT_H = 0x3B,
            TEMP_OUT_H = 0x41,
            GYRO_XOUT_H = 0x43,
            USER_CTRL = 0x6A,
            PWR_MGMT_1 = 0x6B,
            PWR_MGMT_2 = 0x6C,
            FIFO_COUNTH = 0x72,
            FIFO_R_W = 0x74,
            WHO_AM_I = 0x75,
        };

        /**
         * @brief Structure representing a register field with its address and mask.
         * @tparam T The type backing the field's register (always uint8_t here, kept as a
         *           template for consistency and to allow wider fields if ever needed).
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

        // PWR_MGMT_1 fields
        static constexpr Field<uint8_t> device_reset{Address::PWR_MGMT_1, 0x80};
        static constexpr Field<uint8_t> sleep{Address::PWR_MGMT_1, 0x40};
        static constexpr Field<uint8_t> cycle{Address::PWR_MGMT_1, 0x20};
        static constexpr Field<uint8_t> temp_disable{Address::PWR_MGMT_1, 0x08};
        static constexpr Field<uint8_t> clock_source{Address::PWR_MGMT_1, 0x07};

        // PWR_MGMT_2 fields
        static constexpr Field<uint8_t> wake_frequency{Address::PWR_MGMT_2, 0xC0};
        static constexpr Field<uint8_t> standby_xa{Address::PWR_MGMT_2, 0x20};
        static constexpr Field<uint8_t> standby_ya{Address::PWR_MGMT_2, 0x10};
        static constexpr Field<uint8_t> standby_za{Address::PWR_MGMT_2, 0x08};
        static constexpr Field<uint8_t> standby_xg{Address::PWR_MGMT_2, 0x04};
        static constexpr Field<uint8_t> standby_yg{Address::PWR_MGMT_2, 0x02};
        static constexpr Field<uint8_t> standby_zg{Address::PWR_MGMT_2, 0x01};

        // CONFIG fields
        static constexpr Field<uint8_t> ext_sync_set{Address::CONFIG, 0x38};
        static constexpr Field<uint8_t> dlpf_cfg{Address::CONFIG, 0x07};

        // GYRO_CONFIG / ACCEL_CONFIG fields
        static constexpr Field<uint8_t> gyro_fs_sel{Address::GYRO_CONFIG, 0x18};
        static constexpr Field<uint8_t> accel_fs_sel{Address::ACCEL_CONFIG, 0x18};

        // FIFO_EN fields
        static constexpr Field<uint8_t> fifo_en_temp{Address::FIFO_EN, 0x80};
        static constexpr Field<uint8_t> fifo_en_gyro_x{Address::FIFO_EN, 0x40};
        static constexpr Field<uint8_t> fifo_en_gyro_y{Address::FIFO_EN, 0x20};
        static constexpr Field<uint8_t> fifo_en_gyro_z{Address::FIFO_EN, 0x10};
        static constexpr Field<uint8_t> fifo_en_accel{Address::FIFO_EN, 0x08};

        // INT_PIN_CFG fields
        static constexpr Field<uint8_t> int_level{Address::INT_PIN_CFG, 0x80};
        static constexpr Field<uint8_t> int_open_drain{Address::INT_PIN_CFG, 0x40};
        static constexpr Field<uint8_t> int_latch{Address::INT_PIN_CFG, 0x20};
        static constexpr Field<uint8_t> int_rd_clear{Address::INT_PIN_CFG, 0x10};

        // INT_ENABLE / INT_STATUS fields
        static constexpr Field<uint8_t> data_ready_enable{Address::INT_ENABLE, 0x01};
        static constexpr Field<uint8_t> data_ready_status{Address::INT_STATUS, 0x01};

        // USER_CTRL fields
        static constexpr Field<uint8_t> fifo_enable{Address::USER_CTRL, 0x40};
        static constexpr Field<uint8_t> fifo_reset{Address::USER_CTRL, 0x04};

        // WHO_AM_I field
        static constexpr Field<uint8_t> who_am_i{Address::WHO_AM_I, 0x7E};
    };

    // =======================================================
    // I2C Communication
    // =======================================================

    bool readBytes(Register::Address reg, uint8_t *buffer, uint8_t length) const;
    bool readByte(Register::Address reg, uint8_t &value) const { return readBytes(reg, &value, 1); }
    bool readWord(Register::Address reg, int16_t &value) const;

    bool writeByte(Register::Address reg, uint8_t value) const;
    bool writeWord(Register::Address reg, int16_t value) const;

    /**
     * @brief Read a value from the specified register field.
     */
    template <typename T>
    T readField(const Register::Field<T> &field) const
    {
        uint8_t raw = 0;

        if (!readByte(field.register_address, raw))
            return static_cast<T>(0);

        return static_cast<T>((raw & field.mask) >> field.shift());
    }

    /**
     * @brief Write a value into the specified register field (read-modify-write).
     */
    template <typename T>
    bool writeField(const Register::Field<T> &field, T value)
    {
        uint8_t current = 0;

        if (!readByte(field.register_address, current))
            return false;

        current &= static_cast<uint8_t>(~field.mask);
        current |= (static_cast<uint8_t>(value) << field.shift()) & field.mask;

        return writeByte(field.register_address, current);
    }
};