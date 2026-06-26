#pragma once

#include <ackermann_msgs/msg/ackermann_drive_stamped.h>
#include <rcl/rcl.h>

#include "network/network.hpp"
#include "network/virtuals/subscriber.hpp"

class SubscriberAckermannCmd : public ISubscriber
{
public:
    SubscriberAckermannCmd(uint32_t timeout_ms = 200)
        : timeout_ms_(timeout_ms),
          last_message_time_(0) {}

    /**
     * Initialize the subscriber and the watchdog timer
     * @param node The microROS node
     * @param executor The rclc executor
     */
    void init(rcl_node_t *node, rclc_executor_t *executor) override
    {
        rcl_subscription_options_t sub_opt = rcl_subscription_get_default_options();
        sub_opt.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        sub_opt.qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

        RCCHECK(rclc_subscription_init(
            &subscription_,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(ackermann_msgs, msg, AckermannDriveStamped),
            "/ackermann_cmd",
            &sub_opt.qos));

        RCCHECK(rclc_executor_add_subscription_with_context(
            executor,
            &subscription_,
            &msg_,
            &SubscriberAckermannCmd::callback,
            (void *)this,
            ON_NEW_DATA));

        last_message_time_ = millis();
    }

    /**
     * Check if a timeout has occurred
     * Called periodically by the watchdog timer
     */
    bool checkTimeout()
    {
        uint32_t now = millis();
        uint32_t elapsed = now - last_message_time_;

        return elapsed > timeout_ms_;
    }

private:
    ackermann_msgs__msg__AckermannDriveStamped msg_;

    uint32_t timeout_ms_;
    uint32_t last_message_time_;

    static void callback(const void *msgin, void *context)
    {
        const ackermann_msgs__msg__AckermannDriveStamped *msg = static_cast<const ackermann_msgs__msg__AckermannDriveStamped *>(msgin);
        SubscriberAckermannCmd *self = static_cast<SubscriberAckermannCmd *>(context);
        IVehicle *vehicle = Network::getVehicle();

        self->last_message_time_ = millis();
        vehicle->executeMotionCommand(msgin);

        // RCUTILS_LOG_DEBUG("Received Speed: %.2f, Angle: %.2f",
        //                   msg->drive.speed,
        //                   msg->drive.steering_angle);
    }
};
