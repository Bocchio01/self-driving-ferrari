#include <Arduino.h>
#include "network/ros.h"

#include "vehicle/vehicle.hpp"
#include "vehicle/kinematics/ackermann.hpp"
#include "vehicle/kinematics/differential.hpp"
#include "vehicle/actuators/steering.hpp"
#include "vehicle/actuators/propulsion.hpp"

#include "network/network.hpp"
#include "network/subscribers/control_cmd.hpp"
// #include "network/publishers/hello_world.hpp"
#include "network/services/arm_disarm.hpp"

Vehicle<KinematicAckermann> vehicle;
Network network;

/* Vehicle actuators ***************************************/
ActuatorSteering actuator_steering(6, 75, 35, 115);
ActuatorPropulsion actuator_propulsion(3, 2, 4, 5, 0, -100, +100);

/* Vehicle sensors *****************************************/
// To be implemented

/* ROS network publishers **********************************/
// PublisherHelloWorld pub_hello_world;
// PublisherSensorGyro pub_sensor_gyro;
// PublisherSensorIMU pub_sensor_imu;
// PublisherSensorLidar pub_sensor_lidar;
// PublisherSensorSonar pub_sensor_sonar;
// PublisherSensorGPS pub_sensor_gps;
// PublisherSensorBattery pub_sensor_battery;

/* ROS network subscribers *********************************/
SubscriberControlCmd sub_control_cmd;

/* ROS network services ************************************/
ServiceArmDisarm srv_arm_disarm;

void setup()
{
    /* Vehicle setup ***************************************/
    vehicle.bindActuatorSteering(actuator_steering);
    vehicle.bindActuatorPropulsion(actuator_propulsion);
    // vehicle.addSensor(gyroscope);

    /* ROS network setup ***********************************/
    // network.addPublisher(pub_hello_world);
    network.addSubscriber(sub_control_cmd);
    network.addService(srv_arm_disarm);

    /* Network-Vehicle binding and initialization **********/
    network.bindVehicle(vehicle);
    network.init(57600);
}

void loop()
{
    // pub_hello_world.publish();
    network.spinOnce();

    sub_control_cmd.checkTimeout();
    actuator_propulsion.update();
    actuator_steering.update();

    delay(50);
}