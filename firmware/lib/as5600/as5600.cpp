#include "as5600.hpp"

AS5600::AS5600(TwoWire *I2C, uint8_t DIR, uint8_t OUT)
    : I2C_(I2C),
      OUT_(OUT),
      DIR_(DIR)
{
}

bool AS5600::begin()
{
    if (this->OUT_ == NO_PIN && this->I2C_ == nullptr)
    {
        this->error_ = Error::INVALID_PIN;
        return false;
    }

    if (this->OUT_ != NO_PIN)
    {
        pinMode(this->OUT_, INPUT);
    }

    if (this->DIR_ != NO_PIN)
    {
        pinMode(this->DIR_, OUTPUT);
        setDirection(Direction::CLOCKWISE);
    }

    update();

    return true;
}

void AS5600::update()
{
    uint32_t previous_angle_timestamp_ = this->angle_timestamp_;
    uint16_t previous_angle = this->angle_;
    int32_t previous_cumulative_angle_ = this->cumulative_angle_;

    uint16_t angle = readAngle();
    uint32_t now = micros();

    this->angle_ = (angle + RESOLUTION - raw_offset_) % RESOLUTION;
    this->angle_timestamp_ = now;

    int32_t delta = this->angle_ - previous_angle;
    if (delta > (RESOLUTION / 2))
        delta -= RESOLUTION;
    else if (delta < -(RESOLUTION / 2))
        delta += RESOLUTION;

    this->cumulative_angle_ += delta;

    uint32_t delta_time = this->angle_timestamp_ - previous_angle_timestamp_;
    int32_t delta_angle = this->cumulative_angle_ - previous_cumulative_angle_;

    if (delta_time > 0)
    {
        this->current_speed_ = (static_cast<float>(delta_angle) * 1e6f) / delta_time;
    }
}

void AS5600::setZero()
{
    this->raw_offset_ = readAngle();

    this->angle_timestamp_ = micros();
    this->angle_ = 0;
    this->cumulative_angle_ = 0;
    this->current_speed_ = 0.0f;
}

void AS5600::setDirection(Direction direction)
{
    this->direction_ = direction;
    if (this->DIR_ != NO_PIN)
    {
        digitalWrite(this->DIR_, this->direction_ == Direction::CLOCKWISE ? LOW : HIGH);
        delay(20); // Delay to allow the AS5600 to stabilize after changing the direction pin state
    }
}

void AS5600::setOffset(float degrees)
{
    int32_t offset = degrees / tickToAngleUnit(1.0f, AngleUnit::DEGREES);
    offset = offset % RESOLUTION;
    if (offset < 0)
    {
        offset += RESOLUTION;
    }
    this->raw_offset_ = offset;
}

// =======================================================
// Structured Register Access
// =======================================================

AS5600::Configuration AS5600::getConfiguration() const
{
    uint16_t raw = readRegister(Register::configuration);

    return {
        static_cast<PowerMode>((raw & Register::power_mode.mask) >> Register::power_mode.shift()),
        static_cast<Hysteresis>((raw & Register::hysteresis.mask) >> Register::hysteresis.shift()),
        static_cast<OutputMode>((raw & Register::output_mode.mask) >> Register::output_mode.shift()),
        static_cast<PWMFrequency>((raw & Register::pwm_frequency.mask) >> Register::pwm_frequency.shift()),
        static_cast<SlowFilter>((raw & Register::slow_filter.mask) >> Register::slow_filter.shift()),
        static_cast<FastFilter>((raw & Register::fast_filter.mask) >> Register::fast_filter.shift()),
        static_cast<Watchdog>((raw & Register::watchdog.mask) >> Register::watchdog.shift())};
}

bool AS5600::setConfiguration(const Configuration &config)
{
    uint16_t current = 0;

    if (!read(Register::Address::CONF, current))
        return false;

    current &= ~(Register::power_mode.mask |
                 Register::hysteresis.mask |
                 Register::output_mode.mask |
                 Register::pwm_frequency.mask |
                 Register::slow_filter.mask |
                 Register::fast_filter.mask |
                 Register::watchdog.mask);

    current |= (static_cast<uint16_t>(config.power_mode) << Register::power_mode.shift());
    current |= (static_cast<uint16_t>(config.hysteresis) << Register::hysteresis.shift());
    current |= (static_cast<uint16_t>(config.output_mode) << Register::output_mode.shift());
    current |= (static_cast<uint16_t>(config.pwm_frequency) << Register::pwm_frequency.shift());
    current |= (static_cast<uint16_t>(config.slow_filter) << Register::slow_filter.shift());
    current |= (static_cast<uint16_t>(config.fast_filter) << Register::fast_filter.shift());
    current |= (static_cast<uint16_t>(config.watchdog) << Register::watchdog.shift());

    return write(Register::Address::CONF, current);
}

AS5600::Status AS5600::getStatus() const
{
    uint8_t raw = 0;

    // Read the raw register byte directly to avoid auto-shifting
    if (!read(Register::Address::STATUS, raw))
    {
        return {false, false, false};
    }

    // Apply absolute masking given that the register is auto-shifted by the readRegister function
    return {
        (raw & Register::magnet_too_close.mask) != 0,
        (raw & Register::magnet_too_far.mask) != 0,
        (raw & Register::magnet_detected.mask) != 0};
}

// =======================================================
// Sensor Measurement
// =======================================================

uint16_t AS5600::readHardwareAngle(Register::Address address)
{
    uint16_t angle = 0;

    if (this->OUT_ != NO_PIN && address == Register::Address::ANGLE)
    {
        uint32_t adcMax = (1UL << adc_resolution_) - 1;
        uint32_t raw = analogRead(OUT_);
        angle = static_cast<uint16_t>((raw * (RESOLUTION - 1)) / adcMax);
    }
    else
    {
        if (!read(address, angle))
            return this->angle_;
    }

    return angle;
}

// =======================================================
// I2C Communication
// =======================================================

bool AS5600::read(Register::Address reg, uint8_t &value) const
{
    if (this->I2C_ == nullptr)
    {
        this->error_ = Error::I2C_READ;
        return false;
    }

    this->I2C_->beginTransmission(this->ADDR_);
    this->I2C_->write(static_cast<uint8_t>(reg));

    if (this->I2C_->endTransmission() != 0)
    {
        this->error_ = Error::I2C_READ;
        return false;
    }

    if (this->I2C_->requestFrom(this->ADDR_, static_cast<uint8_t>(1)) != 1)
    {
        this->error_ = Error::I2C_READ;
        return false;
    }

    value = this->I2C_->read();
    this->error_ = Error::NONE;

    return true;
}

bool AS5600::read(Register::Address reg, uint16_t &value) const
{
    if (this->I2C_ == nullptr)
    {
        this->error_ = Error::I2C_READ;
        return false;
    }

    this->I2C_->beginTransmission(this->ADDR_);
    this->I2C_->write(static_cast<uint8_t>(reg));

    if (this->I2C_->endTransmission() != 0)
    {
        this->error_ = Error::I2C_READ;
        return false;
    }

    if (this->I2C_->requestFrom(this->ADDR_, static_cast<uint8_t>(2)) != 2)
    {
        this->error_ = Error::I2C_READ;
        return false;
    }

    value = (static_cast<uint16_t>(this->I2C_->read()) << 8) | this->I2C_->read();
    this->error_ = Error::NONE;

    return true;
}

bool AS5600::write(Register::Address reg, uint8_t value) const
{
    if (this->I2C_ == nullptr)
    {
        this->error_ = Error::I2C_WRITE;
        return false;
    }

    this->I2C_->beginTransmission(this->ADDR_);
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

bool AS5600::write(Register::Address reg, uint16_t value) const
{
    if (this->I2C_ == nullptr)
    {
        this->error_ = Error::I2C_WRITE;
        return false;
    }

    this->I2C_->beginTransmission(this->ADDR_);
    this->I2C_->write(static_cast<uint8_t>(reg));
    this->I2C_->write(static_cast<uint8_t>(value >> 8));
    this->I2C_->write(static_cast<uint8_t>(value & 0xFF));

    if (this->I2C_->endTransmission() != 0)
    {
        this->error_ = Error::I2C_WRITE;
        return false;
    }

    this->error_ = Error::NONE;

    return true;
}