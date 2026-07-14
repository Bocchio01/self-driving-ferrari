#include <Arduino.h>
#include <Wire.h>
#include "as5600.hpp"

// Instantiate the encoder.
AS5600 encoder(&Wire, AS5600::NO_PIN, AS5600::NO_PIN);

void setup()
{
    Serial.begin(115200);

    // 1. Initialize the I2C bus BEFORE calling encoder.begin()
    Wire.begin();

    // Wait for the serial port to connect (useful for boards with native USB)
    while (!Serial)
        delay(10);

    Serial.println("Initializing AS5600 Encoder...");

    // 2. Initialize the sensor
    if (!encoder.begin())
    {
        Serial.println("Failed to initialize AS5600! Check wiring and I2C pull-ups.");
        while (1)
        {
            delay(10);
        } // Halt execution
    }

    // 3. (Optional but recommended) Check the magnet status
    AS5600::Status status = encoder.getStatus();
    if (!status.magnetDetected)
    {
        Serial.println("ERROR: No magnet detected!");
    }
    else if (status.magnetTooClose)
    {
        Serial.println("WARNING: Magnet is too close to the sensor.");
    }
    else if (status.magnetTooFar)
    {
        Serial.println("WARNING: Magnet is too far from the sensor.");
    }
    else
    {
        Serial.println("SUCCESS: Magnet detected and at the optimal distance.");
    }

    // 4. Set the current physical position as the software "zero" point
    encoder.setZero();
    Serial.println("Zero position set. Starting readings...");
    Serial.println("--------------------------------------------------");
}

void loop()
{
    // 1. Update the internal state (crucial for speed and cumulative calculations)
    encoder.update();

    // 2. Fetch the calculated values
    uint16_t currentAngle = encoder.getAngle();                       // Range: [0, 4095]
    int32_t cumulativeAngle = encoder.getCumulativeAngle();           // Tracks multi-turn rotations
    float speedRPM = encoder.getAngularSpeed(AS5600::SpeedUnit::RPM); // Speed in RPM

    // 3. Print the results to the Serial Monitor
    Serial.print("Angle: ");
    Serial.print(currentAngle);
    Serial.print("\t | Cumulative: ");
    Serial.print(cumulativeAngle);
    Serial.print("\t | Speed (RPM): ");
    Serial.println(speedRPM);

    // Read at roughly 10Hz.
    // Note: If you decrease this delay too much without adjusting I2C speed,
    // you may choke the bus.
    delay(100);
}