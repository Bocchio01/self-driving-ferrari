#pragma once

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

/**
 * Base class for microROS subscribers
 */
class ISubscriber
{
public:
    virtual ~ISubscriber() = default;

    /**
     * Initialize subscriber in the microROS network
     * Override this in derived classes to set up the subscriber
     */
    virtual void init(rcl_node_t *node, rclc_executor_t *executor) = 0;

    /**
     * Get the subscription handle (for executor registration)
     */
    rcl_subscription_t *getSubscription()
    {
        return &subscription_;
    }

protected:
    rcl_subscription_t subscription_;
};
