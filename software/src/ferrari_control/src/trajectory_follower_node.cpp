#include <ros/ros.h>
#include "trajectory_follower.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_follower");
    TrajectoryFollower trajectory_follower;
    ros::spin();
    
    return 0;
}
