
#include <Arduino.h>
#include <Wire.h>
#include <pid.hpp>

#include "vehicle/configs.hpp"
#define private public
#define protected public
#include "vehicle/actuators/propulsion.hpp"
#undef private
#undef protected
#include "sensors/rotary_encoder.hpp"

PID controller_angular_velocity(071.8f / 100.0f, 043.1f / 100.0f, 000.0f / 100.0f);
PID controller_angular_position(575.0f / 100.0f, 000.0f / 100.0f, 014.0f / 100.0f);
vehicle::actuators::Propulsion actuator_propulsion(29, 30, 31, 32);

sensors::RotaryEncoder encoder(&Wire2, 26);

void PWMToSpeedCalibration();
void speedControllerTuning();
void positionControllerTuning();

void setup()
{
    Serial.begin(115200);
    Wire2.begin();
    // Wire2.setClock(400000); // 400kHz I2C clock speed, not needed in theory

    delay(50);

    Serial.printf("###############################\n");
    Serial.printf("Propulsion Calibration\n");
    Serial.printf("###############################\n");

    actuator_propulsion.reset();
    actuator_propulsion.arm();
    actuator_propulsion.setControllerAngularVelocity(&controller_angular_velocity, []()
                                                     { return encoder.data().angular_speed / vehicle::configs::Kinematic::BELT_RATIO; });
    actuator_propulsion.setControllerAngularPosition(&controller_angular_position, []()
                                                     { return encoder.data().cumulative_angle / vehicle::configs::Kinematic::BELT_RATIO; });

    encoder.begin();
    encoder.setDirection(AS5600::Direction::COUNTER_CLOCKWISE);
    encoder.setZero();

    Serial.printf("\nStarting Calibration...\n");
    delay(3000);
}

void loop()
{
    String input = "";
    static bool repeat = true;

    Serial.clear();
    while (repeat)
    {
        Serial.printf("\nSelect calibration to perform:\n");
        Serial.printf("1. PWM to Speed Calibration\n");
        Serial.printf("2. Speed Controller Tuning\n");
        Serial.printf("3. Position Controller Tuning\n");
        Serial.printf("Choice (1, 2, 3): ");
        while (!Serial.available())
        {
            delay(1);
        }

        input = Serial.readStringUntil('\n').trim();
        switch (input.charAt(0))
        {
        case '1':
            PWMToSpeedCalibration();
            break;
        case '2':
            speedControllerTuning();
            break;
        case '3':
            positionControllerTuning();
            break;
        default:
            Serial.println("Invalid option. Please select 1, 2, or 3.");
            break;
        }

        Serial.printf("\nCalibration completed.\n");
        Serial.printf("Do you want to repeat the calibration process? (y/n): ");
        while (!Serial.available())
        {
            delay(1);
        }

        input = Serial.readStringUntil('\n').trim();
        repeat = input.equalsIgnoreCase("y");
    }

    Serial.printf("\nExiting calibration loop. No further calibrations will be performed.");
    delay(1000);
}

/**
 * @brief Calibrates the relationship between PWM values and speed for the propulsion actuator.
 *
 * Pay attention at aliasing effect that might occur if the sampling frequency is too low compared
 * to the maximum speed of the vehicle. The sampling frequency should be at least twice the maximum
 * frequency of the signal we want to measure (Nyquist theorem).
 *
 * Use this to compute the sampling frequency required based on the maximum speed of the vehicle:
 * ```python
import math
R = 0.0575 / 2.0 # Wheel radius in meters
T = 0.75 # Belt ratio encoder/wheel
compute_sampling_frequency = lambda max_speed: (max_speed * T) / (R * math.pi)
compute_sampling_frequency(4.185) # ~ 35Hz
 * ```
 *
 * For the Ferrari, we have a maximum speed of 4.185 m/s (~15 km/h):
 *  PWM | Speed (rad/s) | Speed (m/s)
 * -150 |       -96.756 |     -2.782
 * -140 |       -95.151 |     -2.736
 * -130 |       -85.172 |     -2.449
 * -120 |       -82.163 |     -2.362
 * -110 |       -74.265 |     -2.135
 * -100 |       -67.369 |     -1.937
 *  -80 |       -54.242 |     -1.559
 *  -70 |       -45.631 |     -1.312
 *  -90 |       -60.166 |     -1.730
 *  -60 |       -37.969 |     -1.092
 *  -50 |       -33.109 |     -0.952
 *  -40 |       -24.093 |     -0.693
 *  -30 |       -18.259 |     -0.525
 *  -20 |       -11.235 |     -0.323
 *  -10 |        -4.003 |     -0.115
 *   +0 |        +0.000 |     +0.000
 *  +10 |        +4.919 |     +0.141
 *  +20 |       +10.945 |     +0.315
 *  +30 |       +18.891 |     +0.543
 *  +40 |       +25.812 |     +0.742
 *  +50 |       +32.479 |     +0.934
 *  +60 |       +34.714 |     +0.998
 *  +70 |       +47.646 |     +1.370
 *  +80 |       +53.757 |     +1.546
 *  +90 |       +61.204 |     +1.760
 * +100 |       +53.602 |     +1.541
 * +110 |       +72.460 |     +2.083
 * +120 |       +81.821 |     +2.352
 * +130 |       +82.338 |     +2.367
 * +140 |       +94.150 |     +2.707
 * +150 |      +103.793 |     +2.984
 * +160 |      +103.694 |     +2.981
 * +170 |      +105.822 |     +3.042
 * +180 |      +110.385 |     +3.174
 * +190 |      +121.941 |     +3.506
 * +200 |      +127.665 |     +3.670
 * +210 |      +130.928 |     +3.764
 * +220 |      +125.867 |     +3.619
 * +230 |      +137.829 |     +3.963
 * +240 |      +127.530 |     +3.666
 * +250 |      +145.579 |     +4.185
 * +255 |      +144.041 |     +4.141
 */
void PWMToSpeedCalibration()
{
    static int8_t direction = +1; // 1 for forward, -1 for reverse

    const uint8_t pmw_values[] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 210, 220, 230, 240, 250, 255};
    const uint8_t N = sizeof(pmw_values) / sizeof(pmw_values[0]);
    float angular_velocity_wheel = 0.0f;
    String input = "";
    uint8_t max_pwm = 255;

    Serial.printf("\nStarting PWM to Speed Calibration...\n");
    Serial.printf("The calibration will run through a series of PWM values and measure the corresponding speed.\n");
    Serial.printf("Please ensure the vehicle is in a safe environment to perform this calibration.\n");
    Serial.printf("Press any key to start the calibration...\n");
    while (!Serial.available())
    {
        delay(100);
    }
    Serial.clear();

    Serial.printf("Enter the maximum PWM value to test (0-255, default 255): ");
    while (!Serial.available())
    {
        delay(1);
    }
    input = Serial.readStringUntil('\n').trim();
    max_pwm = input.toInt();

    Serial.printf("Press 'q' to stop the calibration at any time.\n");
    Serial.printf(" PWM | Speed (rad/s) | Speed (m/s)\n");
    for (uint8_t i = 0; i < N; ++i)
    {
        uint8_t pwm = pmw_values[i];
        if (pwm > max_pwm)
        {
            break;
        }

        digitalWrite(actuator_propulsion.AIN1, direction * pwm > 0 ? HIGH : LOW);
        digitalWrite(actuator_propulsion.AIN2, direction * pwm < 0 ? HIGH : LOW);
        analogWrite(actuator_propulsion.PWMA, abs(pwm));

        uint32_t start_time = millis();
        while (millis() - start_time < 2000)
        {
            if (Serial.available())
            {
                Serial.clear();
                Serial.printf("Calibration stopped by user at PWM value %d.\n", pwm);
                digitalWrite(actuator_propulsion.AIN2, LOW);
                digitalWrite(actuator_propulsion.AIN1, LOW);
                analogWrite(actuator_propulsion.PWMA, 0);
                return;
            }
            encoder.update();
            delay(1e3 / 50); // Sample at ~50Hz
        }

        angular_velocity_wheel = encoder.data().angular_speed / vehicle::configs::Kinematic::BELT_RATIO;
        Serial.printf("%+3d | %+12.3f | %+10.3f\n", direction * pwm, angular_velocity_wheel, angular_velocity_wheel * vehicle::configs::Kinematic::WHEEL_RADIUS);
    }

    // We decelerate the vehicle to a stop after the calibration is complete
    for (uint8_t pwm = max_pwm; pwm > 0; pwm -= 5)
    {
        digitalWrite(actuator_propulsion.AIN1, direction * pwm > 0 ? HIGH : LOW);
        digitalWrite(actuator_propulsion.AIN2, direction * pwm < 0 ? HIGH : LOW);
        analogWrite(actuator_propulsion.PWMA, abs(pwm));
        delay(100);
    }

    digitalWrite(actuator_propulsion.AIN2, LOW);
    digitalWrite(actuator_propulsion.AIN1, LOW);
    analogWrite(actuator_propulsion.PWMA, 0);

    Serial.printf("Repeat also the calibration in opposite direction? (y/n): ");
    while (!Serial.available())
    {
        delay(1);
    }
    input = Serial.readStringUntil('\n').trim();
    if (input.equalsIgnoreCase("y"))
    {
        direction = -direction;
        Serial.printf("Repeating calibration in opposite direction...\n");
        PWMToSpeedCalibration();
    }

    delay(1000);
}

void speedControllerTuning()
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
            String input = Serial.readStringUntil('\n').trim();

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

        encoder.update();
        actuator_propulsion.update();

        if (millis() - last_time > 100)
        {
            Serial.printf("Speed: %.2f/%.2f (C/T), PWM: %d (FF/PID: %d/%+.2f)[%.2f,%.2f,%.2f]\n",
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

        if (Serial.available())
        {
            String input = Serial.readStringUntil('\n').trim();
            if (input.equalsIgnoreCase("q"))
            {
                Serial.printf("Speed controller tuning stopped by user.\n");
                actuator_propulsion.reset();
                break;
            }
        }

        delay(1e3 / 50); // 50Hz update rate -> Max 6m/s to avoid aliasing effects on the encoder
    }
}

void positionControllerTuning()
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
            String input = Serial.readStringUntil('\n').trim();

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

        encoder.update();
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

        if (Serial.available())
        {
            String input = Serial.readStringUntil('\n').trim();
            if (input.equalsIgnoreCase("q"))
            {
                Serial.printf("Position controller tuning stopped by user.\n");
                actuator_propulsion.reset();
                break;
            }
        }

        delay(1e3 / 50); // 50Hz update rate -> Max 6m/s to avoid aliasing effects on the encoder
    }
}