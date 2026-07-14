#pragma once

#include <std_srvs/srv/trigger.h>
#include <stdio.h> // For snprintf

#include "network/network.hpp"
#include "network/virtuals/service.hpp"
#include "vehicle/configs.hpp"

// Here we make the hypothesis that the vehicle is <Ackermann> type
class ServiceGetKinematicLimits : public IService
{
public:
    ServiceGetKinematicLimits() = default;

    /**
     * Initialize the service
     */
    void init(rcl_node_t *node, rclc_executor_t *executor) override
    {
        RCCHECK(rclc_service_init_default(
            &service_,
            node,
            ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
            "/vehicle/get_kinematic_limits"));

        RCCHECK(rclc_executor_add_service_with_context(
            executor,
            &service_,
            &req_,
            &resp_,
            &ServiceGetKinematicLimits::callback,
            (void *)this));
    }

private:
    std_srvs__srv__Trigger_Request req_;
    std_srvs__srv__Trigger_Response resp_;

    static void callback(const void *req, void *resp, void *context)
    {
        std_srvs__srv__Trigger_Response *response = (std_srvs__srv__Trigger_Response *)resp;

        // Static buffer to avoid dynamic memory allocation.
        static char limit_buffer[256];

        snprintf(limit_buffer, sizeof(limit_buffer),
                 "{\"max_speed\": %.3f, \"max_steering\": %.3f, \"wheelbase\": %.3f, \"track_width\": %.3f}",
                 vehicle::configs::Kinematic::MAX_SPEED,
                 vehicle::configs::Kinematic::MAX_STEERING_ANGLE,
                 vehicle::configs::Kinematic::WHEELBASE,
                 vehicle::configs::Kinematic::TRACK_WIDTH);

        response->success = true;
        response->message.data = limit_buffer;
        response->message.size = strlen(response->message.data);
    }
};