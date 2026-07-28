#include "mpu6050.hpp"

MPU6050::MPU6050(TwoWire *I2C, uint8_t address)
    : I2C_(I2C),
      address_(address)
{
}

bool MPU6050::begin(AccelRange accel_range, GyroRange gyro_range)
{
    if (this->I2C_ == nullptr)
    {
        this->error_ = Error::I2C_READ;
        return false;
    }

    setClockSource(ClockSource::PLL_X_GYRO);
    setAccelRange(accel_range);
    setGyroRange(gyro_range);
    setSleepEnabled(false);

    return testConnection();
}

bool MPU6050::testConnection() const
{
    return getDeviceID() == WHO_AM_I_VALUE;
}

void MPU6050::reset()
{
    writeField(Register::device_reset, static_cast<uint8_t>(1));
    delay(100); // allow the device to settle after a full reset
}

// =======================================================
// Power Management
// =======================================================

void MPU6050::setSleepEnabled(bool enabled)
{
    writeField(Register::sleep, static_cast<uint8_t>(enabled));
}

bool MPU6050::getSleepEnabled() const
{
    return readField(Register::sleep) != 0;
}

void MPU6050::setClockSource(ClockSource source)
{
    writeField(Register::clock_source, static_cast<uint8_t>(source));
}

MPU6050::ClockSource MPU6050::getClockSource() const
{
    return static_cast<ClockSource>(readField(Register::clock_source));
}

void MPU6050::setCycleEnabled(bool enabled)
{
    writeField(Register::cycle, static_cast<uint8_t>(enabled));
}

bool MPU6050::getCycleEnabled() const
{
    return readField(Register::cycle) != 0;
}

void MPU6050::setTemperatureSensorEnabled(bool enabled)
{
    // The register bit is a *disable* flag, inverted here for an intuitive API.
    writeField(Register::temp_disable, static_cast<uint8_t>(!enabled));
}

bool MPU6050::getTemperatureSensorEnabled() const
{
    return readField(Register::temp_disable) == 0;
}

void MPU6050::setLowPowerWakeFrequency(LowPowerWakeFrequency frequency)
{
    writeField(Register::wake_frequency, static_cast<uint8_t>(frequency));
}

MPU6050::LowPowerWakeFrequency MPU6050::getLowPowerWakeFrequency() const
{
    return static_cast<LowPowerWakeFrequency>(readField(Register::wake_frequency));
}

// =======================================================
// Full Scale Range Configuration
// =======================================================

void MPU6050::setGyroRange(GyroRange range)
{
    this->gyro_range_ = range;
    writeField(Register::gyro_fs_sel, static_cast<uint8_t>(range));
}

void MPU6050::setAccelRange(AccelRange range)
{
    this->accel_range_ = range;
    writeField(Register::accel_fs_sel, static_cast<uint8_t>(range));
}

float MPU6050::accelResolution() const
{
    switch (this->accel_range_)
    {
    case AccelRange::G_2:
        return 2.0f / 32768.0f;
    case AccelRange::G_4:
        return 4.0f / 32768.0f;
    case AccelRange::G_8:
        return 8.0f / 32768.0f;
    case AccelRange::G_16:
        return 16.0f / 32768.0f;
    default:
        return 2.0f / 32768.0f;
    }
}

float MPU6050::gyroResolution() const
{
    switch (this->gyro_range_)
    {
    case GyroRange::DPS_250:
        return 250.0f / 32768.0f;
    case GyroRange::DPS_500:
        return 500.0f / 32768.0f;
    case GyroRange::DPS_1000:
        return 1000.0f / 32768.0f;
    case GyroRange::DPS_2000:
        return 2000.0f / 32768.0f;
    default:
        return 250.0f / 32768.0f;
    }
}

// =======================================================
// Sample Rate / Filtering
// =======================================================

void MPU6050::setSampleRateDivider(uint8_t divider)
{
    writeByte(Register::Address::SMPLRT_DIV, divider);
}

uint8_t MPU6050::getSampleRateDivider() const
{
    uint8_t value = 0;
    readByte(Register::Address::SMPLRT_DIV, value);
    return value;
}

void MPU6050::setDLPFBandwidth(DLPFBandwidth bandwidth)
{
    writeField(Register::dlpf_cfg, static_cast<uint8_t>(bandwidth));
}

MPU6050::DLPFBandwidth MPU6050::getDLPFBandwidth() const
{
    return static_cast<DLPFBandwidth>(readField(Register::dlpf_cfg));
}

// =======================================================
// Data-Ready Interrupt
// =======================================================

void MPU6050::setInterruptPinConfig(bool active_low, bool open_drain, bool latch, bool clear_on_any_read)
{
    writeField(Register::int_level, static_cast<uint8_t>(active_low));
    writeField(Register::int_open_drain, static_cast<uint8_t>(open_drain));
    writeField(Register::int_latch, static_cast<uint8_t>(latch));
    writeField(Register::int_rd_clear, static_cast<uint8_t>(clear_on_any_read));
}

void MPU6050::setDataReadyInterruptEnabled(bool enabled)
{
    writeField(Register::data_ready_enable, static_cast<uint8_t>(enabled));
}

bool MPU6050::getDataReadyInterruptEnabled() const
{
    return readField(Register::data_ready_enable) != 0;
}

bool MPU6050::getDataReadyInterruptStatus() const
{
    return readField(Register::data_ready_status) != 0;
}

// =======================================================
// Sensor Measurement
// =======================================================

MPU6050::Axis3<int16_t> MPU6050::readRawAcceleration() const
{
    uint8_t buffer[6] = {0};
    Axis3<int16_t> raw{0, 0, 0};

    if (!readBytes(Register::Address::ACCEL_XOUT_H, buffer, 6))
        return raw;

    raw.x = static_cast<int16_t>((buffer[0] << 8) | buffer[1]) - this->accel_offset_.x;
    raw.y = static_cast<int16_t>((buffer[2] << 8) | buffer[3]) - this->accel_offset_.y;
    raw.z = static_cast<int16_t>((buffer[4] << 8) | buffer[5]) - this->accel_offset_.z;

    return raw;
}

MPU6050::Axis3<int16_t> MPU6050::readRawGyroscope() const
{
    uint8_t buffer[6] = {0};
    Axis3<int16_t> raw{0, 0, 0};

    if (!readBytes(Register::Address::GYRO_XOUT_H, buffer, 6))
        return raw;

    raw.x = static_cast<int16_t>((buffer[0] << 8) | buffer[1]);
    raw.y = static_cast<int16_t>((buffer[2] << 8) | buffer[3]);
    raw.z = static_cast<int16_t>((buffer[4] << 8) | buffer[5]);

    return raw;
}

int16_t MPU6050::readRawTemperature() const
{
    int16_t raw = 0;
    readWord(Register::Address::TEMP_OUT_H, raw);
    return raw;
}

MPU6050::Axis3<float> MPU6050::readAcceleration(AccelUnit unit) const
{
    Axis3<int16_t> raw = readRawAcceleration();
    float resolution = accelResolution();

    Axis3<float> result{
        static_cast<float>(raw.x) * resolution,
        static_cast<float>(raw.y) * resolution,
        static_cast<float>(raw.z) * resolution};

    if (unit == AccelUnit::MPS2)
    {
        constexpr float STANDARD_GRAVITY = 9.80665f;
        result.x *= STANDARD_GRAVITY;
        result.y *= STANDARD_GRAVITY;
        result.z *= STANDARD_GRAVITY;
    }

    return result;
}

MPU6050::Axis3<float> MPU6050::readGyroscope(GyroUnit unit) const
{
    Axis3<int16_t> raw = readRawGyroscope();
    float resolution = gyroResolution();

    Axis3<float> result{
        static_cast<float>(raw.x) * resolution,
        static_cast<float>(raw.y) * resolution,
        static_cast<float>(raw.z) * resolution};

    if (unit == GyroUnit::RPS)
    {
        // constexpr float DEG_TO_RAD = PI / 180.0f; // Already defined by Teensyduino
        result.x *= DEG_TO_RAD;
        result.y *= DEG_TO_RAD;
        result.z *= DEG_TO_RAD;
    }

    return result;
}

float MPU6050::readTemperature(TempUnit unit) const
{
    // Per the MPU-6000/MPU-6050 Register Map: Temp_degC = raw / 340 + 36.53
    float celsius = (static_cast<float>(readRawTemperature()) / 340.0f) + 36.53f;

    if (unit == TempUnit::FAHRENHEIT)
        return celsius * 9.0f / 5.0f + 32.0f;

    return celsius;
}

void MPU6050::update()
{
    uint8_t buffer[14] = {0};

    if (!readBytes(Register::Address::ACCEL_XOUT_H, buffer, 14))
        return;

    int16_t raw_ax = static_cast<int16_t>((buffer[0] << 8) | buffer[1]) - this->accel_offset_.x;
    int16_t raw_ay = static_cast<int16_t>((buffer[2] << 8) | buffer[3]) - this->accel_offset_.y;
    int16_t raw_az = static_cast<int16_t>((buffer[4] << 8) | buffer[5]) - this->accel_offset_.z;
    int16_t raw_temp = static_cast<int16_t>((buffer[6] << 8) | buffer[7]);
    int16_t raw_gx = static_cast<int16_t>((buffer[8] << 8) | buffer[9]);
    int16_t raw_gy = static_cast<int16_t>((buffer[10] << 8) | buffer[11]);
    int16_t raw_gz = static_cast<int16_t>((buffer[12] << 8) | buffer[13]);

    float accel_res = accelResolution();
    float gyro_res = gyroResolution();

    this->acceleration_.x = static_cast<float>(raw_ax) * accel_res;
    this->acceleration_.y = static_cast<float>(raw_ay) * accel_res;
    this->acceleration_.z = static_cast<float>(raw_az) * accel_res;

    this->gyroscope_.x = static_cast<float>(raw_gx) * gyro_res;
    this->gyroscope_.y = static_cast<float>(raw_gy) * gyro_res;
    this->gyroscope_.z = static_cast<float>(raw_gz) * gyro_res;

    this->temperature_ = (static_cast<float>(raw_temp) / 340.0f) + 36.53f;
    this->timestamp_ = micros();
}

// =======================================================
// Offset Calibration
// =======================================================

void MPU6050::setGyroOffset(Axis3<int16_t> offset)
{
    writeWord(Register::Address::XG_OFFS_USRH, offset.x);
    writeWord(Register::Address::YG_OFFS_USRH, offset.y);
    writeWord(Register::Address::ZG_OFFS_USRH, offset.z);
}

MPU6050::Axis3<int16_t> MPU6050::getGyroOffset() const
{
    Axis3<int16_t> offset{0, 0, 0};

    readWord(Register::Address::XG_OFFS_USRH, offset.x);
    readWord(Register::Address::YG_OFFS_USRH, offset.y);
    readWord(Register::Address::ZG_OFFS_USRH, offset.z);

    return offset;
}

void MPU6050::setAccelOffset(Axis3<int16_t> offset)
{
    // Held in RAM only, subtracted from every raw accel reading — see the notes on
    // setAccelOffset() in the header for why this isn't written to the device.
    this->accel_offset_ = offset;
}

// =======================================================
// FIFO Buffer
// =======================================================

void MPU6050::setFIFOBufferEnabled(bool enabled)
{
    writeField(Register::fifo_enable, static_cast<uint8_t>(enabled));
}

bool MPU6050::getFIFOBufferEnabled() const
{
    return readField(Register::fifo_enable) != 0;
}

void MPU6050::resetFIFOBuffer()
{
    writeField(Register::fifo_reset, static_cast<uint8_t>(1));
}

void MPU6050::setFIFOEnabledSources(bool temperature, bool accel, bool gyro_x, bool gyro_y, bool gyro_z)
{
    uint8_t value = 0;

    if (temperature)
        value |= Register::fifo_en_temp.mask;
    if (gyro_x)
        value |= Register::fifo_en_gyro_x.mask;
    if (gyro_y)
        value |= Register::fifo_en_gyro_y.mask;
    if (gyro_z)
        value |= Register::fifo_en_gyro_z.mask;
    if (accel)
        value |= Register::fifo_en_accel.mask;

    writeByte(Register::Address::FIFO_EN, value);
}

uint16_t MPU6050::getFIFOCount() const
{
    uint8_t buffer[2] = {0};

    if (!readBytes(Register::Address::FIFO_COUNTH, buffer, 2))
        return 0;

    return static_cast<uint16_t>((buffer[0] << 8) | buffer[1]);
}

void MPU6050::readFIFOBytes(uint8_t *data, uint8_t length) const
{
    if (length == 0)
        return;

    readBytes(Register::Address::FIFO_R_W, data, length);
}

// =======================================================
// Identification / Error Handling
// =======================================================

uint8_t MPU6050::getDeviceID() const
{
    return readField(Register::who_am_i);
}

uint8_t MPU6050::readRegister(uint8_t address) const
{
    uint8_t value = 0;
    readBytes(static_cast<Register::Address>(address), &value, 1);
    return value;
}

bool MPU6050::writeRegister(uint8_t address, uint8_t value) const
{
    return writeByte(static_cast<Register::Address>(address), value);
}

// =======================================================
// I2C Communication
// =======================================================

bool MPU6050::readBytes(Register::Address reg, uint8_t *buffer, uint8_t length) const
{
    if (this->I2C_ == nullptr)
    {
        this->error_ = Error::I2C_READ;
        return false;
    }

    this->I2C_->beginTransmission(this->address_);
    this->I2C_->write(static_cast<uint8_t>(reg));

    if (this->I2C_->endTransmission(false) != 0)
    {
        this->error_ = Error::I2C_READ;
        return false;
    }

    if (this->I2C_->requestFrom(this->address_, length) != length)
    {
        this->error_ = Error::I2C_READ;
        return false;
    }

    for (uint8_t i = 0; i < length; i++)
        buffer[i] = static_cast<uint8_t>(this->I2C_->read());

    this->error_ = Error::NONE;

    return true;
}

bool MPU6050::readWord(Register::Address reg, int16_t &value) const
{
    uint8_t buffer[2] = {0};

    if (!readBytes(reg, buffer, 2))
        return false;

    value = static_cast<int16_t>((buffer[0] << 8) | buffer[1]);

    return true;
}

bool MPU6050::writeByte(Register::Address reg, uint8_t value) const
{
    if (this->I2C_ == nullptr)
    {
        this->error_ = Error::I2C_WRITE;
        return false;
    }

    this->I2C_->beginTransmission(this->address_);
    this->I2C_->write(static_cast<uint8_t>(reg));
    this->I2C_->write(value);

    if (this->I2C_->endTransmission() != 0)
    {
        this->error_ = Error::I2C_WRITE;
        return false;
    }

    this->error_ = Error::NONE;

    return true;
}

bool MPU6050::writeWord(Register::Address reg, int16_t value) const
{
    if (this->I2C_ == nullptr)
    {
        this->error_ = Error::I2C_WRITE;
        return false;
    }

    this->I2C_->beginTransmission(this->address_);
    this->I2C_->write(static_cast<uint8_t>(reg));
    this->I2C_->write(static_cast<uint8_t>((value >> 8) & 0xFF));
    this->I2C_->write(static_cast<uint8_t>(value & 0xFF));

    if (this->I2C_->endTransmission() != 0)
    {
        this->error_ = Error::I2C_WRITE;
        return false;
    }

    this->error_ = Error::NONE;

    return true;
}