#pragma once

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

/**
 * Base class for microROS services
 */
class IService
{
public:
    virtual ~IService() = default;

    /**
     * Initialize service in the microROS network
     */
    virtual void init(rcl_node_t *node, rclc_executor_t *executor) = 0;

    /**
     * Get the service handle for executor registration
     */
    rcl_service_t *getService() { return &service_; }

protected:
    rcl_service_t service_;
};
