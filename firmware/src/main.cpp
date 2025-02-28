#include <Arduino.h>
#include <ros.h>

#include "vehicle/vehicle.hpp"
#include "vehicle/kinematics/bicycle.hpp"
#include "vehicle/kinematics/differential.hpp"
#include "vehicle/actuators/servo.hpp"
#include "vehicle/actuators/motor.hpp"

#include "network/network.hpp"
// #include "network/services/vehicle_info.hpp"
#include "network/subscribers/turtle1_cmd_vel.hpp"
// #include "network/publishers/hello_world.hpp"

/* Function prototypes */
void setup();
void loop();

/* Vehicle */
MyServo servo(9);
Motor motor(10, 2, 3);
Vehicle<Bicycle> vehicle(servo, motor);

/* Network */
SubscriberTurtle1CmdVel subscriberTurtle1CmdVel;
// PublisherHelloWorld publisherHelloWorld;
Network network;


void setup()
{
    network.addSubscriber(subscriberTurtle1CmdVel);
    // network.addPublisher(publisherHelloWorld);
    
    servo.arm();
    motor.arm();
    network.bindVehicle(vehicle);
    network.init(57600);
}

void loop()
{
    // publisherHelloWorld.publish();
    network.spinOnce();
    delay(10);
}