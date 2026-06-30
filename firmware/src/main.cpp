#include <Arduino.h>

#include "vehicle/vehicle.hpp"
#include "vehicle/kinematics/ackermann.hpp"
#include "vehicle/actuators/steering.hpp"
#include "vehicle/actuators/propulsion.hpp"

#include "network/network.hpp"
#include "network/subscribers/ackermann_cmd.hpp"
#include "network/services/toggle_engage_vehicle.hpp"

Vehicle<KinematicAckermann> vehicle;
Network &network = Network::getInstance();

// Actuators
ActuatorSteering actuator_steering(2, 75, 35, 115);
ActuatorPropulsion actuator_propulsion(29, 30, 31, 32, 0, -100, +100);

// Network components
SubscriberAckermannCmd sub_control_cmd;
ServiceToggleEngageVehicle srv_toggle_engage_vehicle;

void setup()
{
    // Serial setup for debugging
    Serial.begin(115200);

    // Vehicle setup
    vehicle.bindActuatorSteering(actuator_steering);
    vehicle.bindActuatorPropulsion(actuator_propulsion);

    // Network setup
    network.addSubscriber(sub_control_cmd);
    network.addService(srv_toggle_engage_vehicle);
    network.bindVehicle(vehicle);
    network.configure(Serial1, 115200, "ferrari_node", RCUTILS_LOG_SEVERITY_INFO);
}

void loop()
{
    network.spin();

    if (sub_control_cmd.checkTimeout())
    {
        vehicle.executeEmercencyStop();
    }

    actuator_propulsion.update();
    actuator_steering.update();

    delay(20);
}