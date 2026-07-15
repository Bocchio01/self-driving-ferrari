/**
 * @file encoder.cpp
 * @brief Calibration script for the AS5600 encoder.
 *
 * This script is used to calibrate the AS5600 encoder connected to the Teensy microcontroller.
 * It performs two main calibration steps: belt ratio calibration and dead zone calibration.
 *
 * The belt ratio calibration determines the ratio between the encoder's revolutions and the actual wheel revolutions.
 * The dead zone calibration identifies the range of motion where the encoder does not respond to small movements due to mechanical slack or backlash.
 *
 * The script uses the Wire library for I2C communication and the Serial library for user interaction and output.
 *
 * @note Ensure that the AS5600 encoder is properly connected to the Teensy before running this script.
 */

#include <Arduino.h>
#include <Wire.h>

#define private public
#define protected public
#include <as5600.hpp>
#undef private
#undef protected

#include "vehicle/configs.hpp"

AS5600 encoder(&Wire2, 26);

void beltRatioCalibration();
void deadZoneCalibration();

void checkEncoderError(const char *context)
{
    AS5600::Error err = encoder.getError();
    if (err != AS5600::Error::NONE)
    {
        Serial.printf(">>> I2C ERROR during %s: Code %u\n", context, static_cast<uint8_t>(err));
        encoder.clearError();
    }
}

void setup()
{
    Serial.begin(115200);
    Wire2.begin();

    while (!Serial)
    {
        delay(100);
    }

    Serial.printf("###############################\n");
    Serial.printf("Encoder Calibration\n");
    Serial.printf("###############################\n");

    while (!encoder.begin())
    {
        checkEncoderError("initialization");
        delay(500);
    }

    Serial.printf("AS5600 initialized successfully at I2C address 0x%02X\n", encoder.ADDR_);

    AS5600::Configuration config = encoder.getConfiguration();
    AS5600::Status status = encoder.getStatus();

    Serial.printf("AS5600 Configuration:\n");
    Serial.printf("\tPower Mode: %u\n", static_cast<uint8_t>(config.power_mode));
    Serial.printf("\tHysteresis: %u\n", static_cast<uint8_t>(config.hysteresis));
    Serial.printf("\tOutput Mode: %u\n", static_cast<uint8_t>(config.output_mode));
    Serial.printf("\tPWM Frequency: %u\n", static_cast<uint8_t>(config.pwm_frequency));
    Serial.printf("\tSlow Filter: %u\n", static_cast<uint8_t>(config.slow_filter));
    Serial.printf("\tFast Filter: %u\n", static_cast<uint8_t>(config.fast_filter));
    Serial.printf("\tWatchdog: %u\n", static_cast<uint8_t>(config.watchdog));

    Serial.printf("\nAS5600 Status:\n");
    Serial.printf("\tMagnet Detected: \t%s\n", status.magnetDetected ? "Yes" : "No");
    Serial.printf("\tMagnet Too Close: \t%s\n", status.magnetTooClose ? "Yes" : "No");
    Serial.printf("\tMagnet Too Far: \t%s\n", status.magnetTooFar ? "Yes" : "No");

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
        Serial.printf("1. Belt Ratio Calibration\n");
        Serial.printf("2. Dead Zone Calibration\n");
        Serial.printf("Choice (1, 2): ");
        while (!Serial.available())
        {
            delay(1);
        }

        input = Serial.readStringUntil('\n').trim();
        switch (input.charAt(0))
        {
        case '1':
            beltRatioCalibration();
            break;
        case '2':
            deadZoneCalibration();
            break;
        default:
            Serial.println("Invalid option. Please select 1 or 2.");
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
 * @brief Calibrates the belt ratio of the encoder.
 *
 * For the Ferrari, we have a belt ratio of 0.75, measured over 100 wheel revolutions
 * corresponding to 75 encoder revolutions.
 */
void beltRatioCalibration()
{
    float belt_ratio = 0.0f;
    float counted_revolutions = 0.0f;
    String input = "";

    Serial.printf("\nBelt Ratio Calibration:\n");
    Serial.printf("This calibration will determine the ratio between the encoder's revolutions and the actual wheel revolutions.\n");
    Serial.printf("Rotate the wheel for a known number of revolutions and enter the value when done.\n");
    Serial.printf("Enter the number of revolutions when done (e.g., 5.0): ");
    while (!Serial.available())
    {
        encoder.update();
        checkEncoderError("update");

        delay(500);
    }

    input = Serial.readStringUntil('\n').trim();
    counted_revolutions = input.toFloat();

    belt_ratio = encoder.getRevolutions() / counted_revolutions;
    Serial.printf("Belt ratio: %5.4f\n", belt_ratio);

    delay(1000);
}

/**
 * @brief Calibrates the dead zone of the encoder.
 *
 * For the Ferrari, we have:
 * - On continuous rotation, no dead zone (elastic belt transmission always in tension)
 * - On change of direction, ~20 degrees of dead zone / backlash (elastic belt transmission slack)
 */
void deadZoneCalibration()
{
    float final_angle = 0.0f;
    float deadzone = 0.0f;
    String input = "";

    Serial.printf("\nDead Zone Calibration:\n");
    Serial.printf("This calibration will determine the dead zone of the encoder where it does not respond to small movements due to mechanical slack or backlash.\n");
    Serial.printf("Move the wheel slowly clockwise to a known position, press Enter, then move it slowly in the positive direction to make a full revolution and press Enter again.\n");
    Serial.printf("The encoder will record the positions and calculate the dead zone.\n");

    // Wait for the user to press Enter to start the calibration
    while (!Serial.available() || Serial.read() != '\n')
    {
        delay(100);
    }

    // Capture the initial position
    encoder.setZero();
    Serial.printf("Move the wheel in the positive direction to make a full revolution and press Enter again.\n");

    // Wait for the user to press Enter again after making a full revolution
    while (!Serial.available())
    {
        encoder.update();
        checkEncoderError("update");
    }
    Serial.clear();

    final_angle = encoder.getAngle(AS5600::AngleUnit::DEGREES) / vehicle::configs::Kinematic::BELT_RATIO;
    deadzone = 360.0f - final_angle;
    Serial.printf("Dead zone: %5.2f degrees\n", deadzone);

    delay(1000);
}