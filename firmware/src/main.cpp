#include <Arduino.h>
#include <Wire.h>
#include <pid.hpp>

#include "vehicle/configs.hpp"
#include "vehicle/vehicle.hpp"
#include "vehicle/kinematics/ackermann.hpp"
#include "vehicle/actuators/steering.hpp"
#include "vehicle/actuators/propulsion.hpp"

#include "sensors/rotary_encoder.hpp"

#include "network/network.hpp"
#include "network/subscribers/ackermann_cmd.hpp"
#include "network/services/toggle_arm_actuators.hpp"
#include "network/publishers/odom.hpp"

// Vehicle
vehicle::Vehicle<vehicle::kinematics::Ackermann> ferrari;
vehicle::kinematics::Ackermann ackermann_kinematics;
vehicle::actuators::Steering actuator_steering(2);
vehicle::actuators::Propulsion actuator_propulsion(29, 30, 31, 32);
PID controller_angular_velocity(071.8f / 100.0f, 043.1f / 100.0f, 000.0f / 100.0f);
PID controller_angular_position(575.0f / 100.0f, 000.0f / 100.0f, 014.0f / 100.0f);

// Sensors
sensors::RotaryEncoder encoder(&Wire2, 26);

// Network
Network &network = Network::getInstance();
SubscriberAckermannCmd sub_control_cmd;
ServiceToggleArmActuators srv_toggle_arm_actuators;
PublisherOdom publisher_odometry(10.0f);

// Helper functions to shift the feedback from the rotary encoder to the center of the rear axle, given that the encoder is mounted on the left wheel.
float feedbackAngularVelocity();
float feedbackAngularPosition();

void setup()
{
    Serial.begin(115200); // Debugging serial port
    Wire2.begin();        // Required by RotaryEncoder

    // Vehicle setup
    actuator_propulsion.setControllerAngularVelocity(&controller_angular_velocity, feedbackAngularVelocity);
    actuator_propulsion.setControllerAngularPosition(&controller_angular_position, feedbackAngularPosition);
    ackermann_kinematics.bindActuators(actuator_steering, actuator_propulsion);
    ferrari.bindKinematics(&ackermann_kinematics);

    // Sensors setup
    encoder.begin();
    encoder.setDirection(AS5600::Direction::COUNTER_CLOCKWISE);
    encoder.setZero();

    // Network setup
    network.addPublisher(publisher_odometry);
    network.addSubscriber(sub_control_cmd);
    network.addService(srv_toggle_arm_actuators);
    network.bindVehicle(ferrari);
    network.configure(Serial1, 1000000, "ferrari_node", RCUTILS_LOG_SEVERITY_INFO);
}

void loop()
{
    static unsigned long last_loop_time = 0;
    unsigned long current_time = millis();
    unsigned long loop_interval = 1000 / vehicle::configs::Actuators::LOOP_RATE;

    if (current_time - last_loop_time >= loop_interval)
    {
        last_loop_time = current_time;

        network.spin();

        if (sub_control_cmd.checkTimeout())
        {
            ferrari.executeEmergencyStop();
        }

        encoder.update();
        ferrari.update();

        if (network.isConnected())
        {
            publisher_odometry.publish();
        }
    }
}

// ########################################################################################
// We need to apply these correction given that the feedback from the rotary encoder is
// relative to the left wheel, while the speed command is relative to the center of the
// rear axle.
// ########################################################################################

float correctionFactor(float steering_angle)
{
    using Kinematic = vehicle::configs::Kinematic;
    return 1.0f - ((Kinematic::TRACK_WIDTH * tan(steering_angle)) / (2.0f * Kinematic::WHEELBASE));
}

float feedbackAngularVelocity()
{
    using Kinematic = vehicle::configs::Kinematic;

    float current_steering_angle = actuator_steering.getCurrentSteeringAngle();
    float left_wheel_angular_velocity = encoder.data().angular_speed / Kinematic::BELT_RATIO;

    // vl = v - (v * tan(delta) / L) * (T / 2) -> v = vl / (1 - (T * tan(delta)) / (2 * L))
    return left_wheel_angular_velocity / correctionFactor(current_steering_angle);
}

float feedbackAngularPosition()
{
    using Kinematic = vehicle::configs::Kinematic;

    static float last_left_rad = 0.0f;
    static float center_cumulative_rad = 0.0f;

    float current_steering_angle = actuator_steering.getCurrentSteeringAngle();
    float current_left_rad = encoder.data().cumulative_angle / Kinematic::BELT_RATIO;

    // Apply the steering correction ONLY to the tiny distance moved in this specific tick
    center_cumulative_rad += (current_left_rad - last_left_rad) / correctionFactor(current_steering_angle);
    last_left_rad = current_left_rad;

    return center_cumulative_rad;
}