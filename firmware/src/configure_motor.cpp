/**
 * Script to configure the ferrari motor
 */
#include <Arduino.h>
#include "vehicle/actuators/propulsion.hpp"

ActuatorPropulsion actuator_propulsion(3, 2, 4, 5);

void setup()
{
    Serial.begin(57600);
    delay(50);

    actuator_propulsion.reset();
    actuator_propulsion.arm();
}

void loop()
{
    if (Serial.available())
    {
        String input = Serial.readStringUntil('\r\n');
        int value = input.toInt();

        actuator_propulsion.setThrottle(constrain(value, -255, +255));

        Serial.print("Throttle target: ");
        Serial.println(value);
    }

    actuator_propulsion.update();

    delay(50);
}