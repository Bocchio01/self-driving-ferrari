#pragma once

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

/**
 * Base class for microROS publishers
 */
class IPublisher
{
public:
    virtual ~IPublisher() = default;

    /**
     * Initialize publisher in the microROS network
     */
    virtual void init(rcl_node_t *node) = 0;

    /**
     * Publish a message
     */
    virtual void publish(const void *msg) = 0;

    /**
     * Get the publisher handle
     */
    rcl_publisher_t *getPublisher()
    {
        return &publisher_;
    }

private:
    rcl_publisher_t publisher_;
};