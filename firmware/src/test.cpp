#include <Arduino.h>
#include <Wire.h>
#include <as5600.hpp>

AS5600 encoder(&Wire2, 26);

// Helper function to print I2C errors
void checkError(const char *context)
{
    AS5600::Error err = encoder.getError();
    if (err != AS5600::Error::NONE)
    {
        Serial.print(">>> I2C ERROR during ");
        Serial.print(context);
        Serial.print(": Code ");
        Serial.println(static_cast<uint8_t>(err));
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

    while (!encoder.begin())
    {
        checkError("initialization");
        delay(500);
    }

    encoder.setDirection(AS5600::Direction::COUNTER_CLOCKWISE);

    // Read configuration and status registers
    AS5600::Configuration config = encoder.getConfiguration();
    AS5600::Status status = encoder.getStatus();

    Serial.println("AS5600 Configuration:");
    Serial.printf("Power Mode: %u\n", static_cast<uint8_t>(config.power_mode));
    Serial.printf("Hysteresis: %u\n", static_cast<uint8_t>(config.hysteresis));
    Serial.printf("Output Mode: %u\n", static_cast<uint8_t>(config.output_mode));
    Serial.printf("PWM Frequency: %u\n", static_cast<uint8_t>(config.pwm_frequency));
    Serial.printf("Slow Filter: %u\n", static_cast<uint8_t>(config.slow_filter));
    Serial.printf("Fast Filter: %u\n", static_cast<uint8_t>(config.fast_filter));
    Serial.printf("Watchdog: %u\n", static_cast<uint8_t>(config.watchdog));

    Serial.println("\nAS5600 Status:");
    Serial.printf("Magnet Detected: \t%s\n", status.magnetDetected ? "Yes" : "No");
    Serial.printf("Magnet Too Close: \t%s\n", status.magnetTooClose ? "Yes" : "No");
    Serial.printf("Magnet Too Far: \t%s\n", status.magnetTooFar ? "Yes" : "No");

    encoder.setZero();

    Serial.println();
    delay(3000);
}

void loop()
{
    encoder.update();

    uint16_t relAngle = encoder.getAngle();
    int32_t revs = encoder.getRevolutions();
    float rpm = encoder.getAngularSpeed(AS5600::SpeedUnit::RPM);

    Serial.printf("Rel: %04u\tRevs: %+02ld\tRPM: %+07.2f\n", relAngle, revs, rpm);

    delay(100);
}