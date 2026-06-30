#pragma once

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl_interfaces/msg/log.h>
#include <rmw_microros/rmw_microros.h>
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

#define RCCHECK(fn)                                                                                  \
    {                                                                                                \
        rcl_ret_t temp_rc = fn;                                                                      \
        if ((temp_rc != RCL_RET_OK))                                                                 \
        {                                                                                            \
            Serial.println("Error in " #fn " at line " + String(__LINE__) + ": " + String(temp_rc)); \
            Serial.println(rcl_get_error_string().str);                                              \
            rcl_reset_error();                                                                       \
            for (uint8_t i = 0; i < 10; ++i)                                                         \
            {                                                                                        \
                digitalWrite(13, !digitalRead(13));                                                  \
                delay(100);                                                                          \
            }                                                                                        \
            return;                                                                                  \
        }                                                                                            \
    }

#define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
        }                            \
    }

// Global variables for logging
rcl_publisher_t rosout_pub;
rcl_interfaces__msg__Log rosout_msg;

class Network
{
public:
    enum class AgentState
    {
        WAITING_AGENT,
        AGENT_AVAILABLE,
        AGENT_CONNECTED,
        AGENT_DISCONNECTED
    };

    static Network &getInstance()
    {
        static Network instance;
        return instance;
    }

    ~Network()
    {
        destroyEntities();
    }

    /**
     * Configure the microROS network
     * @param serial Serial port to use for microROS communication
     * @param serial_baudrate Serial port baud rate (typically 115200)
     * @param node_name Name of the ROS2 node
     * @param log_level Logging level for microROS (default: RCUTILS_LOG_SEVERITY_DEBUG)
     */
    template <typename SerialClass>
    void configure(
        SerialClass &serial,
        uint32_t serial_baudrate = 115200,
        const char *node_name = "teensy_node",
        RCUTILS_LOG_SEVERITY log_level = RCUTILS_LOG_SEVERITY_DEBUG)
    {
        // Configure the serial transport for microROS
        serial.begin(serial_baudrate);
        set_microros_serial_transports(serial);

        // Set the node name for microROS
        node_name_ = node_name;

        // Set the logging level and output handler for microROS
        rcutils_logging_set_default_logger_level(log_level);
        rcutils_logging_set_output_handler(Network::rosout_log_handler);

        // Allocate memory for rosout message fields
        rosout_msg.msg.data = (char *)malloc(128 * sizeof(char));
        rosout_msg.msg.capacity = 128;

        rosout_msg.name.data = (char *)malloc(32 * sizeof(char));
        rosout_msg.name.capacity = 32;

        rosout_msg.file.data = (char *)malloc(32 * sizeof(char));
        rosout_msg.file.capacity = 32;

        rosout_msg.function.data = (char *)malloc(32 * sizeof(char));
        rosout_msg.function.capacity = 32;

        state_ = AgentState::WAITING_AGENT;
    }

    /**
     * Spin the microROS network. This method should be called in the main loop.
     * It handles the state machine for connecting to the microROS agent and processing callbacks.
     * @param timeout_ms Timeout in milliseconds for processing callbacks (default: 10ms)
     */
    void spin(uint32_t timeout_ms = 10)
    {
        Serial.println("Network::spin: state=" + String(static_cast<int>(state_)));
        switch (state_)
        {
        case AgentState::WAITING_AGENT:
            if (rmw_uros_ping_agent(200, 5) == RMW_RET_OK)
            {
                state_ = AgentState::AGENT_AVAILABLE;
            }
            break;

        case AgentState::AGENT_AVAILABLE:
            createEntities();
            if (isInitialized())
            {
                state_ = AgentState::AGENT_CONNECTED;
            }
            else
            {
                destroyEntities();
                state_ = AgentState::WAITING_AGENT;
            }
            break;

        case AgentState::AGENT_CONNECTED:
            if (rmw_uros_ping_agent(10, 1) == RMW_RET_OK)
            {
                rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(timeout_ms));
            }
            else
            {
                state_ = AgentState::AGENT_DISCONNECTED;
            }
            break;

        case AgentState::AGENT_DISCONNECTED:
            destroyEntities();
            state_ = AgentState::WAITING_AGENT;
            break;
        }
    }

    bool isInitialized() const
    {
        return initialized_;
    }

    bool isConnected() const
    {
        return state_ == AgentState::AGENT_CONNECTED;
    }

    // Registration methods
    void addSubscriber(ISubscriber &subscriber) { subscribers_[subscriber_count_++] = &subscriber; }
    void addService(IService &service) { services_[service_count_++] = &service; }
    void addPublisher(IPublisher &publisher) { publishers_[publisher_count_++] = &publisher; }

    // Vehicle binding methods
    void bindVehicle(IVehicle &vehicle) { vehicle_ = &vehicle; }
    static IVehicle *getVehicle() { return getInstance().vehicle_; }

    // Accessors
    rcl_node_t *getNode() { return &node_; }
    rclc_executor_t *getExecutor() { return &executor_; }
    rcl_allocator_t *getAllocator() { return &allocator_; }

private:
    Network() = default;
    Network(const Network &) = delete;
    Network &operator=(const Network &) = delete;

    AgentState state_ = AgentState::WAITING_AGENT;
    bool initialized_ = false;
    const char *node_name_ = "";

    IVehicle *vehicle_ = nullptr;

    rcl_allocator_t allocator_;
    rclc_support_t support_;
    rcl_node_t node_;
    rclc_executor_t executor_;

    ISubscriber *subscribers_[MAX_SUBSCRIBERS];
    IService *services_[MAX_SERVICES];
    IPublisher *publishers_[MAX_PUBLISHERS];

    uint8_t subscriber_count_ = 0;
    uint8_t service_count_ = 0;
    uint8_t publisher_count_ = 0;

    /**
     * Builds all micro-ROS entities.
     * This method is called when the network is connected to the micro-ROS agent.
     * It initializes the node, executor, and all registered publishers, subscribers, and services.
     */
    void createEntities()
    {
        allocator_ = rcl_get_default_allocator();

        RCCHECK(rclc_support_init(&support_, 0, NULL, &allocator_));
        RCCHECK(rclc_node_init_default(&node_, node_name_, "", &support_));
        RCCHECK(rclc_executor_init(&executor_, &support_.context, MAX_SUBSCRIBERS + MAX_SERVICES, &allocator_));

        // Initialize rosout publisher
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

        // Initialize user-registered entities
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

        initialized_ = true;
    }

    /**
     * Destroys all micro-ROS entities.
     * This method is called when the network is disconnected or when the Network object is destroyed.
     */
    void destroyEntities()
    {
        initialized_ = false;

        rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support_.context);
        if (!rmw_context)
            return; // Nothing to destroy

        for (uint8_t i = 0; i < publisher_count_; ++i)
        {
            RCSOFTCHECK(rcl_publisher_fini(publishers_[i]->getPublisher(), &node_));
        }
        for (uint8_t i = 0; i < subscriber_count_; ++i)
        {
            RCSOFTCHECK(rcl_subscription_fini(subscribers_[i]->getSubscription(), &node_));
        }
        for (uint8_t i = 0; i < service_count_; ++i)
        {
            RCSOFTCHECK(rcl_service_fini(services_[i]->getService(), &node_));
        }

        RCSOFTCHECK(rcl_publisher_fini(&rosout_pub, &node_));

        RCSOFTCHECK(rclc_executor_fini(&executor_));
        RCSOFTCHECK(rcl_node_fini(&node_));
        RCSOFTCHECK(rclc_support_fini(&support_));
    }

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
    }
};