#include <Arduino.h>
// #include <stdio.h>
#include <ros.h>

#include "vehicle/vehicle.hpp"
#include "vehicle/kinematics/bicycle.hpp"
#include "vehicle/kinematics/differential.hpp"
#include "vehicle/actuators/servo.hpp"
#include "vehicle/actuators/motor.hpp"

#include "network/network.hpp"
// #include "network/services/vehicle_info.hpp"
#include "network/publishers/hello_world.hpp"

#define DEBUG 0
#define SERIAL_DEBUG if (DEBUG) Serial


/* Function prototypes */
void setup();
void loop();
int serial_putc(char c, FILE *);


/* Vehicle */
MyServo servo(9);
Motor motor(10, 2, 3);
Vehicle<Bicycle> vehicle(servo, motor);

/* Network */
PublisherHelloWorld publisherHelloWorld;
Network network;


void setup()
{
    SERIAL_DEBUG.begin(57600);
    fdevopen(&serial_putc, 0);

    network.addPublisher(publisherHelloWorld);
    network.init();
}

void loop()
{
    publisherHelloWorld.publish();
    network.spinOnce();
    delay(500);
}

int serial_putc(char c, FILE *)
{
    SERIAL_DEBUG.write(c);
    return c;
}