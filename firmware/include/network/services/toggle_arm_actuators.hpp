#pragma once

#include <std_srvs/srv/trigger.h>

#include "network/network.hpp"
#include "network/virtuals/service.hpp"

class ServiceToggleArmActuators : public IService
{
public:
    ServiceToggleArmActuators() = default;

    /**
     * Initialize the service
     */
    void init(rcl_node_t *node, rclc_executor_t *executor) override
    {
        RCCHECK(rclc_service_init_default(
            &service_,
            node,
            ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
            "/vehicle/toggle_arm_actuators"));

        RCCHECK(rclc_executor_add_service_with_context(
            executor,
            &service_,
            &req_,
            &resp_,
            &ServiceToggleArmActuators::callback,
            (void *)this));
    }

private:
    std_srvs__srv__Trigger_Request req_;
    std_srvs__srv__Trigger_Response resp_;

    static void callback(const void *req, void *resp, void *context)
    {
        std_srvs__srv__Trigger_Response *response = (std_srvs__srv__Trigger_Response *)resp;
        // ServiceToggleArmActuators *self = (ServiceToggleArmActuators *)context;

        vehicle::interfaces::Vehicle *vehicle = Network::getVehicle();

        if (vehicle->isArmed())
        {
            response->success = vehicle->disarm();
            response->message.data = (char *)"Vehicle disarmed successfully.";
        }
        else
        {
            response->success = vehicle->arm();
            response->message.data = (char *)"Vehicle armed successfully.";
        }

        response->message.size = strlen(response->message.data);
    }
};