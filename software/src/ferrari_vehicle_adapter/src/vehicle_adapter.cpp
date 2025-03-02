#include "vehicle_adapter.hpp"
#include "kinematic_adapter_interface.hpp"
#include "ferrari_common/get_kinematic_model.h"
#include "ackermann_adapter.hpp"
#include "differential_drive_adapter.hpp"

VehicleAdapter::VehicleAdapter()
{
    // ros::service::waitForService("get_kinematic_model");
    this->srv_get_kinematic_model = this->nh.serviceClient<ferrari_common::get_kinematic_model>("get_kinematic_model");
    // this->kinematic_adapter = nullptr;

    // this->queryVehicleKinematicModel();
    this->kinematic_adapter = new AckermannAdapter(this->nh);
}

VehicleAdapter::~VehicleAdapter()
{
}

void VehicleAdapter::queryVehicleKinematicModel()
{
    ferrari_common::get_kinematic_model get_kinematic_model;
    this->srv_get_kinematic_model.call(get_kinematic_model);

    switch (get_kinematic_model.response.kinematic_model)
    {

    case static_cast<int>(KinematicModel::ACKERMANN):
        this->kinematic_adapter = new AckermannAdapter(this->nh);
        break;

    case static_cast<int>(KinematicModel::DIFFERENTIAL_DRIVE):
        this->kinematic_adapter = new DifferentialDriveAdapter(this->nh);
        break;

        // case static_cast<int>(KinematicModel::OMNIDIRECTIONAL):
        //     this->kinematic_adapter = new OmnidirectionalAdapter(this->nh);
        //     break;

    default:
        ROS_ERROR("Unknown kinematic model");
        break;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vehicle_adapter");
    VehicleAdapter vehicle_adapter;
    ros::spin();
}