#pragma once

#include <Arduino.h>

class PID
{
public:
    explicit PID(double kp = 0, double ki = 0, double kd = 0)
        : output_min_(-1e6), output_max_(1e6)
    {
        this->setGains(kp, ki, kd);
        this->reset();
    }

    double compute(double setpoint, double measured_value)
    {
        uint32_t now = micros();
        double dt = (now - this->last_time_) / 1000000.0; // Convert microseconds to seconds
        this->last_time_ = now;

        // To prevent large jumps in the output when the PID is first initialized or
        // when the time difference is too large, we can skip the first computation
        if (this->first_pass_ || (now - this->last_time_) > 100000)
        {
            this->last_time_ = now;
            this->previous_measured_value_ = measured_value;
            this->first_pass_ = false;
            return this->output_;
        }

        double error = setpoint - measured_value;
        this->integral_ += error * dt;
        this->integral_ = constrain(this->integral_, this->output_min_, this->output_max_);
        double derivative = dt > 1e-6 ? -(measured_value - this->previous_measured_value_) / dt : 0;
        this->previous_error_ = error;
        this->previous_measured_value_ = measured_value;

        double output = this->kp_ * error + this->ki_ * this->integral_ + this->kd_ * derivative;
        this->output_ = constrain(output, this->output_min_, this->output_max_);

        return this->output_;
    }

    void reset()
    {
        this->last_time_ = micros();
        this->integral_ = 0.0;
        this->previous_error_ = 0.0;
        this->previous_measured_value_ = 0.0;
        this->output_ = 0.0;
        this->first_pass_ = true;
    }

    void setGains(double kp = 0, double ki = 0, double kd = 0)
    {
        this->kp_ = kp;
        this->ki_ = ki;
        this->kd_ = kd;
    }

    void setOutputLimits(double min, double max)
    {
        if (min >= max)
        {
            return;
        }

        this->output_min_ = min;
        this->output_max_ = max;
        this->integral_ = constrain(this->integral_, this->output_min_, this->output_max_);
    }

    void setKp(double kp) { this->kp_ = kp; }
    void setKi(double ki) { this->ki_ = ki; }
    void setKd(double kd) { this->kd_ = kd; }
    double getKp() const { return this->kp_; }
    double getKi() const { return this->ki_; }
    double getKd() const { return this->kd_; }

    double getOutput() const { return this->output_; }

private:
    bool first_pass_ = true;
    uint32_t last_time_;
    double kp_, ki_, kd_;

    double integral_;
    double previous_error_;
    double previous_measured_value_;

    double output_, output_min_, output_max_;
};