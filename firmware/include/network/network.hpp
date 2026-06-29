#pragma once

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl_interfaces/msg/log.h>
#include <rcutils/time.h>
#include <rcutils/logging.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

#include "vehicle/vehicle.hpp"
#include "network/virtuals/publisher.hpp"
#include "network/virtuals/subscriber.hpp"
#include "network/virtuals/service.hpp"

#define MAX_SUBSCRIBERS 8
#define MAX_SERVICES 8
#define MAX_PUBLISHERS 8

// Error handling macros
#define RCCHECK(fn)                              \
    {                                            \
        rcl_ret_t temp_rc = fn;                  \
        if ((temp_rc != RCL_RET_OK))             \
        {                                        \
            for (int i = 0; i < 10; i++)         \
            {                                    \
                digitalWrite(LED_BUILTIN, HIGH); \
                delay(100);                      \
                digitalWrite(LED_BUILTIN, LOW);  \
                delay(100);                      \
            }                                    \
            SCB_AIRCR = 0x05FA0004;              \
        }                                        \
    }

#define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
        }                            \
    }

// Wrapper to convert Arduino millis() into the format ROS 2 requires
rcutils_ret_t get_arduino_time_ns(rcutils_time_point_value_t *now)
{
    if (now == nullptr)
    {
        return RCUTILS_RET_INVALID_ARGUMENT;
    }

    // Convert Arduino milliseconds to ROS nanoseconds
    *now = (rcutils_time_point_value_t)millis() * 1000000LL;

    return RCUTILS_RET_OK;
}

// Global or class member variables
rcl_publisher_t rosout_pub;
rcl_interfaces__msg__Log rosout_msg;

class Network
{
public:
    static Network &getInstance()
    {
        static Network instance;
        return instance;
    }

    ~Network()
    {
        // Finalize all registered publishers, services, and subscribers
        for (uint8_t i = 0; i < publisher_count_; ++i)
        {
            RCCHECK(rcl_publisher_fini(publishers_[i]->getPublisher(), &node_));
        }

        for (uint8_t i = 0; i < subscriber_count_; ++i)
        {
            RCCHECK(rcl_subscription_fini(subscribers_[i]->getSubscription(), &node_));
        }

        for (uint8_t i = 0; i < service_count_; ++i)
        {
            RCCHECK(rcl_service_fini(services_[i]->getService(), &node_));
        }

        // Finalize the rosout publisher and free its memory
        RCCHECK(rcl_publisher_fini(&rosout_pub, &node_));
        free(rosout_msg.msg.data);
        free(rosout_msg.name.data);

        // Finalize core micro-ROS components in the correct order!
        RCCHECK(rclc_executor_fini(&executor_));
        RCCHECK(rcl_node_fini(&node_));
        RCCHECK(rclc_support_fini(&support_));
    }

    /**
     * Initialize the microROS network
     * @param serial Serial port to use for microROS communication
     * @param serial_baudrate Serial port baud rate (typically 115200)
     * @param node_name Name of the ROS2 node
     * @param log_level Logging level for microROS (default: RCUTILS_LOG_SEVERITY_DEBUG)
     */
    template <typename SerialClass>
    void init(
        SerialClass &serial,
        uint32_t serial_baudrate = 115200,
        const char *node_name = "teensy_node",
        RCUTILS_LOG_SEVERITY log_level = RCUTILS_LOG_SEVERITY_DEBUG)
    {
        if (isInitialized())
        {
            return;
        }

        // Initialize the serial transport for microROS
        serial.begin(serial_baudrate);
        set_microros_serial_transports(serial);

        // Wait for the serial connection to be established
        delay(2000);

        // Initialize logging
        rcutils_logging_set_default_logger_level(log_level);
        rcutils_logging_set_output_handler(Network::rosout_log_handler);

        // Initialize microROS internals
        allocator_ = rcl_get_default_allocator();
        RCCHECK(rclc_support_init(&support_, 0, NULL, &allocator_));
        RCCHECK(rclc_node_init_default(&node_, node_name, "", &support_));
        RCCHECK(rclc_executor_init(&executor_, &support_.context, MAX_SUBSCRIBERS + MAX_SERVICES, &allocator_));

        // Initialize the rosout publisher for logging
        rcl_publisher_options_t pub_opt = rcl_publisher_get_default_options();
        pub_opt.qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        pub_opt.qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
        pub_opt.qos.depth = 10;

        RCCHECK(rclc_publisher_init(
            &rosout_pub,
            &node_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
            "/rosout",
            &pub_opt.qos));

        rosout_msg.msg.data = (char *)malloc(128 * sizeof(char));
        rosout_msg.msg.capacity = 128;

        rosout_msg.name.data = (char *)malloc(32 * sizeof(char));
        rosout_msg.name.capacity = 32;

        rosout_msg.file.data = (char *)malloc(32 * sizeof(char));
        rosout_msg.file.capacity = 32;

        rosout_msg.function.data = (char *)malloc(32 * sizeof(char));
        rosout_msg.function.capacity = 32;

        // Initialize all registered publishers, services, and subscribers
        for (uint8_t i = 0; i < publisher_count_; ++i)
        {
            publishers_[i]->init(&node_);
        }

        for (uint8_t i = 0; i < service_count_; ++i)
        {
            services_[i]->init(&node_, &executor_);
        }

        for (uint8_t i = 0; i < subscriber_count_; ++i)
        {
            subscribers_[i]->init(&node_, &executor_);
        }

        // Mark the network as initialized
        initialized_ = true;
    }

    /**
     * Register a subscriber with the network
     */
    void addSubscriber(ISubscriber &subscriber)
    {
        subscribers_[subscriber_count_++] = &subscriber;
    }

    /**
     * Register a service with the network
     */
    void addService(IService &service)
    {
        services_[service_count_++] = &service;
    }

    /**
     * Register a publisher with the network
     */
    void addPublisher(IPublisher &publisher)
    {
        publishers_[publisher_count_++] = &publisher;
    }

    /**
     * Bind a vehicle to the network
     */
    void bindVehicle(IVehicle &vehicle)
    {
        vehicle_ = &vehicle;
    }

    /**
     * Get the bound vehicle
     */
    static IVehicle *getVehicle()
    {
        Network &network = getInstance();
        return network.vehicle_;
    }

    /**
     * Process one iteration of the executor (non-blocking)
     * @param timeout_ms Timeout in milliseconds for executor spin
     */
    void spinOnce(uint32_t timeout_ms = 10)
    {
        if (!isInitialized())
        {
            return;
        }

        RCSOFTCHECK(rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(timeout_ms)));
    }

    // Accessors for internal components
    rcl_node_t *getNode() { return &node_; }
    rclc_executor_t *getExecutor() { return &executor_; }
    rcl_allocator_t *getAllocator() { return &allocator_; }

    // Check if the network has been initialized
    bool isInitialized() const { return initialized_; }

private:
    Network() = default;

    // Prevent copying
    Network(const Network &) = delete;
    Network &operator=(const Network &) = delete;

    // Pointer to the bound vehicle
    IVehicle *vehicle_ = nullptr;

    // microROS components
    rcl_allocator_t allocator_;
    rclc_support_t support_;
    rcl_node_t node_;
    rclc_executor_t executor_;

    // Registered network components
    ISubscriber *subscribers_[MAX_SUBSCRIBERS];
    IService *services_[MAX_SERVICES];
    IPublisher *publishers_[MAX_PUBLISHERS];

    uint8_t subscriber_count_ = 0;
    uint8_t service_count_ = 0;
    uint8_t publisher_count_ = 0;
    bool initialized_ = false;

    static void rosout_log_handler(
        const rcutils_log_location_t *location,
        int severity,
        const char *name,
        rcutils_time_point_value_t timestamp,
        const char *format,
        va_list *args)
    {
        vsnprintf(rosout_msg.msg.data, rosout_msg.msg.capacity, format, *args);
        rosout_msg.msg.size = strlen(rosout_msg.msg.data);

        const char *logger_name = name ? name : "micro_ros_node";
        snprintf(rosout_msg.name.data, rosout_msg.name.capacity, "%s", logger_name);
        rosout_msg.name.size = strlen(rosout_msg.name.data);

        switch (severity)
        {
        case RCUTILS_LOG_SEVERITY_DEBUG:
            rosout_msg.level = rcl_interfaces__msg__Log__DEBUG;
            break;
        case RCUTILS_LOG_SEVERITY_INFO:
            rosout_msg.level = rcl_interfaces__msg__Log__INFO;
            break;
        case RCUTILS_LOG_SEVERITY_WARN:
            rosout_msg.level = rcl_interfaces__msg__Log__WARN;
            break;
        case RCUTILS_LOG_SEVERITY_ERROR:
            rosout_msg.level = rcl_interfaces__msg__Log__ERROR;
            break;
        case RCUTILS_LOG_SEVERITY_FATAL:
            rosout_msg.level = rcl_interfaces__msg__Log__FATAL;
            break;
        default:
            rosout_msg.level = rcl_interfaces__msg__Log__INFO;
            break;
        }

        RCSOFTCHECK(rcl_publish(&rosout_pub, &rosout_msg, NULL));
        // printf("[%s]: %s\n", logger_name, rosout_msg.msg.data);
    }
};
