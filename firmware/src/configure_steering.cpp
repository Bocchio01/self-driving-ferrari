#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcutils/logging.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/string.h>

#include "vehicle/actuators/steering.hpp"

ActuatorSteering actuator_steering(23);

rcl_subscription_t subscriber;
std_msgs__msg__Int8 msg;

rcl_publisher_t log_publisher;
std_msgs__msg__String log_msg;
char log_buffer[100];

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Error handling macros
#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            error_loop();            \
        }                            \
    }
#define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
        }                            \
    }

void error_loop()
{
    while (1)
    {
        delay(100);
    }
}

// Custom log function to publish strings
void publish_log(const char *message)
{
    // 1. Write directly into the memory allocated for the ROS 2 String
    // snprintf is safe and returns the exact number of characters written
    int chars_written = snprintf(
        log_msg.data.data,
        log_msg.data.capacity,
        "%s",
        message);

    // 2. Explicitly set the size for the micro-ROS serializer
    log_msg.data.size = chars_written;

    RCCHECK(rcl_publish(&log_publisher, &log_msg, NULL));
}

void subscription_callback(const void *msgin)
{
    // Cast the incoming message
    const std_msgs__msg__Int8 *incoming_msg = (const std_msgs__msg__Int8 *)msgin;

    // Log the received steering command
    snprintf(log_buffer, sizeof(log_buffer), "Received steering command: %d", incoming_msg->data);
    publish_log(log_buffer);

    actuator_steering.setTargetSteeringAngle(incoming_msg->data);
}
void setup()
{
    // 1. Hardware initialization
    actuator_steering.reset();
    actuator_steering.arm();

    // 2. micro-ROS transport initialization
    Serial.begin(115200); // Standard micro-ROS baud rate
    set_microros_serial_transports(Serial);

    delay(2000); // Give the micro-ROS agent time to connect

    allocator = rcl_get_default_allocator();

    // 3. Initialize support and node
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "ferrari_steering_config_node", "", &support));

    // 4. Initialize subscriber
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        "steering_target"));

    RCCHECK(rclc_publisher_init_default(
        &log_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "teensy_logs"));

    log_msg.data.data = log_buffer;
    log_msg.data.capacity = sizeof(log_buffer);
    log_msg.data.size = 0;

    // 5. Initialize executor with 1 handle (our subscriber)
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop()
{
    // Spin the executor to process any incoming ROS 2 messages
    // The timeout (10ms) allows it to yield back to the main loop quickly
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

    // Update the physical actuator state
    actuator_steering.update();

    delay(20);
}