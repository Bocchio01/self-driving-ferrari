/**
 * @file calibration_steering.cpp
 * @brief Calibration script for the steering actuator.
 *
 * This script is used to calibrate the steering actuator connected to the Teensy microcontroller.
 * It performs a calibration step to determine the relationship between PWM values and steering angles.
 *
 * @note Ensure that the steering actuator is properly connected to the Teensy before running this script.
 */

#include <Arduino.h>
#include <Wire.h>
#include <pid.hpp>

#include "vehicle/configs.hpp"
#define private public
#define protected public
#include "vehicle/actuators/steering.hpp"
#undef private
#undef protected

vehicle::actuators::Steering actuator_steering(2);

void PWMToSteeringCalibration();

void setup()
{
    Serial.begin(115200);

    delay(50);

    Serial.printf("###############################\n");
    Serial.printf("Steering Calibration\n");
    Serial.printf("###############################\n");

    actuator_steering.reset();
    actuator_steering.arm();

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
        Serial.printf("1. PWM to Steering Calibration\n");
        Serial.printf("Choice (1): ");
        while (!Serial.available())
        {
            delay(1);
        }

        input = Serial.readStringUntil('\n').trim();
        switch (input.charAt(0))
        {
        case '1':
            PWMToSteeringCalibration();
            break;
        default:
            Serial.println("Invalid option. Please select 1.");
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
 * @brief Calibrates the relationship between PWM values and steering angles.
 *
 * For Ferrari, we have (straight steering at ~75PWM):
 * PWM | Steering Angle (rad) | Steering Angle (deg) | Turning Radius (m)
 *  35 |             -0.35421 |            -20.29464 |            -0.5895
 *  40 |             -0.32429 |            -18.58060 |            -0.6485
 *  50 |             -0.23426 |            -13.42217 |            -0.9135
 *  60 |             -0.14171 |             -8.11960 |            -1.5280
 *  90 |              0.14085 |              8.07010 |             1.5375
 * 100 |              0.23957 |             13.72617 |             0.8925
 * 110 |              0.31479 |             18.03609 |             0.6695
 * 115 |              0.35201 |             20.16892 |             0.5935
 *
 * Moreover, the servo mounted has a range from +17PWM to +167PWM (150PWM range).
 */
void PWMToSteeringCalibration()
{
    uint8_t n = 0;
    const uint8_t N = 10;
    uint8_t pmw_values[N] = {75}; // Default starting PWM value (straight steering)
    float steering_angle_values[N] = {0};
    float turning_radius[N] = {0};
    String input = "";

    Serial.printf("\n\nStarting PWM to Steering Calibration...\n");
    Serial.printf("You will be prompted to enter a series of PWM values and the corresponding turning radii (in meters) for the steering.\n");
    Serial.printf("The calibration will help determine the relationship between PWM values and steering angles.\n");
    Serial.printf("Please ensure the vehicle is in a safe environment to perform this calibration.\n");
    Serial.printf("During the calibration, you can enter:\n");
    Serial.printf("  - 'r' to revert to the previous PWM value (e.g., out of range PWM value).\n");
    Serial.printf("  - 'q' to end the calibration process.\n");
    Serial.printf("Press any key to start the calibration...\n");
    while (!Serial.available())
    {
        delay(1);
    }
    Serial.clear();

    while (n < N)
    {
        // Prompt for PWM value
        Serial.printf("PWM (or 'q'): ");
        while (!Serial.available())
        {
            delay(1);
        }
        input = Serial.readStringUntil('\n').trim();
        if (input.equalsIgnoreCase("q"))
            break;

        pmw_values[n] = input.toInt();
        actuator_steering.write(pmw_values[n]);

        // Prompt for turning radius
        Serial.printf("Turning Radius [m] (or 'r'): ");
        while (!Serial.available())
        {
            delay(1);
        }
        input = Serial.readStringUntil('\n').trim();
        if (input.equalsIgnoreCase("r"))
        {
            if (n > 0)
            {
                n--;
                actuator_steering.write(pmw_values[n]);
            }
        }
        else
        {
            turning_radius[n] = input.toFloat();
        }

        n++;
    }

    Serial.printf("\nPWM | Steering Angle (rad) | Steering Angle (deg) | Turning Radius (m)\n");
    for (uint8_t i = 0; i < n; i++)
    {
        steering_angle_values[i] = atan(vehicle::configs::Kinematic::WHEELBASE / turning_radius[i]);
        Serial.printf("%3d | %20.5f | %20.5f | %18.4f\n", pmw_values[i], steering_angle_values[i], steering_angle_values[i] * 180.0 / M_PI, turning_radius[i]);
    }

    delay(1000);
}