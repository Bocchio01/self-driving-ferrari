/**
 * Script to configure the ferrari motor
 */
#include <Arduino.h>
#include <Wire.h>
#include <pid.hpp>
#include "vehicle/configs.hpp"
#define private public
#include "vehicle/actuators/propulsion.hpp"
#undef private
#include "sensors/rotary_encoder.hpp"

PID controller_angular_velocity(071.8f / 100.0f, 043.1f / 100.0f, 000.0f / 100.0f);
PID controller_angular_position(575.0f / 100.0f, 000.0f / 100.0f, 014.0f / 100.0f);
vehicle::actuators::Propulsion actuator_propulsion(29, 30, 31, 32);
sensors::RotaryEncoder rotary_encoder(&Wire2, 26);

void encoderCalibration();
void PWMToSpeedCalibration();
void SpeedControllerTuning();
void PositionControllerTuning();

void setup()
{
    Serial.begin(57600);
    Wire2.begin();
    Wire2.setClock(400000); // 400kHz I2C clock speed

    delay(50);

    actuator_propulsion.reset();
    actuator_propulsion.arm();
    actuator_propulsion.setControllerAngularVelocity(&controller_angular_velocity, []()
                                                     { return rotary_encoder.data().angular_speed / vehicle::configs::Kinematic::BELT_RATIO; });
    actuator_propulsion.setControllerAngularPosition(&controller_angular_position, []()
                                                     { return rotary_encoder.data().cumulative_angle / vehicle::configs::Kinematic::BELT_RATIO; });

    rotary_encoder.begin();
    rotary_encoder.setDirection(AS5600::Direction::COUNTER_CLOCKWISE);
    rotary_encoder.setZero();
}

void loop()
{
    // encoderCalibration();
    // PWMToSpeedCalibration();
    SpeedControllerTuning();
    // PositionControllerTuning();
}

void encoderCalibration()
{
    while (!Serial.available())
    {
        rotary_encoder.update();

        float angle = rotary_encoder.getAngle(AS5600::AngleUnit::DEGREES);
        float revolutions = rotary_encoder.getRevolutions();
        float angular_speed = rotary_encoder.getAngularSpeed(AS5600::SpeedUnit::RADIANS_PER_SECOND);

        Serial.printf("Revolutions: %5.2f | Angular speed: %5.2f rad/s | Angle: %5.2f deg\n", revolutions, angular_speed, angle);

        delay(500);
    }

    String input = Serial.readStringUntil('\r\n');
    float counted_revolutions = input.toFloat();

    float belt_ratio = counted_revolutions / rotary_encoder.getRevolutions();
    Serial.printf("Belt ratio: %5.4f\n", belt_ratio);

    delay(1000);
}

void PWMToSpeedCalibration()
{
    /*
    To avoid aliasing effects, we need to sample the encoder at a frequency at least twice the maximum frequency of the signal we want to measure.
    Use this to compute the sampling frequency required based on the maximum speed of the vehicle:
import math
R = 0.0575 / 2.0 # Wheel radius in meters
T = 0.75 # Belt ratio encoder/wheel
compute_sampling_frequency = lambda max_speed: (max_speed * T) / (R * math.pi)
compute_sampling_frequency(8.33) # ~ 70Hz
    */

    /*
    PWM:    0 | Speed:   0.00 rad/s | 0.00 m/s
    PWM:   10 | Speed:   0.00 rad/s | 0.00 m/s
    PWM:   20 | Speed:  14.08 rad/s | 0.54 m/s
    PWM:   30 | Speed:  26.26 rad/s | 1.01 m/s
    PWM:   40 | Speed:  36.95 rad/s | 1.42 m/s
    PWM:   50 | Speed:  45.63 rad/s | 1.75 m/s
    PWM:   60 | Speed:  62.23 rad/s | 2.38 m/s
    PWM:   70 | Speed:  71.99 rad/s | 2.76 m/s
    PWM:   80 | Speed:  84.43 rad/s | 3.23 m/s
    PWM:   90 | Speed:  98.78 rad/s | 3.79 m/s
    PWM:  100 | Speed: 112.44 rad/s | 4.31 m/s
    PWM:  110 | Speed: 121.32 rad/s | 4.65 m/s
    PWM:  120 | Speed: 128.32 rad/s | 4.92 m/s
    PWM:  130 | Speed: 138.16 rad/s | 5.30 m/s
    PWM:  140 | Speed: 151.08 rad/s | 5.79 m/s
    PWM:  150 | Speed: 169.70 rad/s | 6.51 m/s
    */

    while (true)
    {
        for (uint8_t pwm = 0; pwm <= 100; pwm += 10)
        {

            digitalWrite(actuator_propulsion.AIN1, pwm > 0 ? HIGH : LOW);
            digitalWrite(actuator_propulsion.AIN2, pwm < 0 ? HIGH : LOW);
            analogWrite(actuator_propulsion.PWMA, abs(pwm));

            uint32_t start_time = millis();
            while (millis() - start_time < 2000)
            {
                rotary_encoder.update();
                delay(1e3 / 70); // Sample at ~70Hz
            }

            float angular_speed = rotary_encoder.data().angular_speed;
            float speed = (angular_speed / vehicle::configs::Kinematic::BELT_RATIO) * vehicle::configs::Kinematic::WHEEL_RADIUS;

            Serial.printf("PWM: %4d | Speed: %5.2f rad/s | %5.2f m/s\n", pwm, angular_speed, speed);
        }

        delay(5000);
    }
}

void SpeedControllerTuning()
{
    uint32_t last_time = millis();

    Serial.println("Commands:");
    Serial.println("  P<val> : Set Kp (e.g., P1.5)");
    Serial.println("  I<val> : Set Ki (e.g., I0.1)");
    Serial.println("  D<val> : Set Kd (e.g., D0.05)");
    Serial.println("  (Or just type a number to set speed)");

    while (true)
    {
        if (Serial.available())
        {
            String input = Serial.readStringUntil('\r\n');

            char command = toupper(input.charAt(0));

            switch (command)
            {
            case 'P':
                actuator_propulsion.controller_angular_velocity->setKp(input.substring(1).toFloat());
                break;

            case 'I':
                actuator_propulsion.controller_angular_velocity->setKi(input.substring(1).toFloat());
                break;

            case 'D':
                actuator_propulsion.controller_angular_velocity->setKd(input.substring(1).toFloat());
                break;

            default:
                actuator_propulsion.setTargetAngularVelocity(input.toFloat() / vehicle::configs::Kinematic::WHEEL_RADIUS);
                break;
            }

            actuator_propulsion.controller_angular_velocity->reset();
        }

        rotary_encoder.update();
        actuator_propulsion.update();

        if (millis() - last_time > 100)
        {
            Serial.printf("Speed: %.2f/%.2f (C/T), PWM: %d (FF/PID: %.2f/%.2f)[%.2f,%.2f,%.2f]\n",
                          actuator_propulsion.current_angular_velocity * vehicle::configs::Kinematic::WHEEL_RADIUS,
                          actuator_propulsion.target_angular_velocity * vehicle::configs::Kinematic::WHEEL_RADIUS,
                          actuator_propulsion.motor_pwm,
                          vehicle::configs::Actuators::ANGULAR_VELOCITY_TO_MOTOR_PWM(actuator_propulsion.target_angular_velocity),
                          actuator_propulsion.controller_angular_velocity->getOutput(),
                          actuator_propulsion.controller_angular_velocity->getKp(),
                          actuator_propulsion.controller_angular_velocity->getKi(),
                          actuator_propulsion.controller_angular_velocity->getKd());
            last_time = millis();
        }

        delay(10); // 100Hz update rate -> Max 12m/s to avoid aliasing effects on the encoder
    }
}

void PositionControllerTuning()
{
    uint32_t last_time = millis();

    Serial.println("Commands:");
    Serial.println("  P<val> : Set Kp (e.g., P1.5)");
    Serial.println("  I<val> : Set Ki (e.g., I0.1)");
    Serial.println("  D<val> : Set Kd (e.g., D0.05)");
    Serial.println("  (Or just type a number to set speed)");

    actuator_propulsion.setTargetAngularVelocity(0.0f);

    while (true)
    {
        if (Serial.available())
        {
            String input = Serial.readStringUntil('\r\n');

            char command = toupper(input.charAt(0));

            switch (command)
            {
            case 'P':
                actuator_propulsion.controller_angular_position->setKp(input.substring(1).toFloat());
                actuator_propulsion.controller_angular_position->reset();
                break;

            case 'I':
                actuator_propulsion.controller_angular_position->setKi(input.substring(1).toFloat());
                actuator_propulsion.controller_angular_position->reset();
                break;

            case 'D':
                actuator_propulsion.controller_angular_position->setKd(input.substring(1).toFloat());
                actuator_propulsion.controller_angular_position->reset();
                break;

            default:
                actuator_propulsion.setTargetAngularPosition(input.toFloat() / vehicle::configs::Kinematic::WHEEL_RADIUS);
                break;
            }
        }

        rotary_encoder.update();
        actuator_propulsion.update();

        if (millis() - last_time > 100)
        {
            Serial.printf("Position [cm]: %.2f/%.2f (C/T), PWM: %d [%.2f,%.2f,%.2f]\n",
                          actuator_propulsion.current_angular_position * vehicle::configs::Kinematic::WHEEL_RADIUS * 100.0f,
                          actuator_propulsion.target_angular_position * vehicle::configs::Kinematic::WHEEL_RADIUS * 100.0f,
                          actuator_propulsion.motor_pwm,
                          actuator_propulsion.controller_angular_position->getKp(),
                          actuator_propulsion.controller_angular_position->getKi(),
                          actuator_propulsion.controller_angular_position->getKd());
        }

        delay(10); // 100Hz update rate -> Max 12m/s to avoid aliasing effects on the encoder
    }
}