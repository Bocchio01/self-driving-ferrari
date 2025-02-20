#include <Arduino.h>
#include <stdio.h>

#include "vehicle/vehicle.hpp"
#include "vehicle/kinematics/bicycle.hpp"
#include "vehicle/kinematics/differential.hpp"
#include "vehicle/actuators/servo.hpp"
#include "vehicle/actuators/motor.hpp"

#include "network/network.hpp"
// // #include "network/services/vehicle_info.hpp"
#include "network/publishers/hello_world.hpp"


/* Vehicle */
Servo servo(9);
Motor motor(10, 2, 3);
Vehicle<Bicycle> vehicle(servo, motor);

/* Network */
// Network network(vehicle);


int serial_putc(char c, FILE *)
{
    Serial.write(c);
    return c;
}

void setup()
{
    Serial.begin(57600);
    // fdevopen(&serial_putc, 0);


    // network.addService(std::make_shared<VehicleInfoService>());
    // network.addPublisher(std::make_shared<HelloWorldPublisher>());

    // network.init();
}

void loop()
{
    // network.spinOnce();
    delay(500);
}

