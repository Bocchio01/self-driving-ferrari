/**
 * Script to configure the ferrari steering
 */
#include <Arduino.h>
#include "vehicle/actuators/steering.hpp"

ActuatorSteering actuator_steering(6);

void setup()
{
    Serial.begin(57600);
    actuator_steering.reset();
    actuator_steering.arm();
}

void loop()
{
    if (Serial.available())
    {
        String input = Serial.readStringUntil('\r\n');
        int8_t value = input.toInt();

        actuator_steering.setSteeringAngle(value);

        Serial.print("Angle target: ");
        Serial.println(value);
    }

    actuator_steering.update();

    delay(50);
}