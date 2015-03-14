#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include <iostream>

#include "grasp_logic/grasp.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grasp_logic_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<grasp_logic::grasp>("grasp_logic");
    grasp_logic::grasp srv;
    geometry_msgs::PoseStamped EE_pose;


    // Stamped Pose
    EE_pose.header.frame_id = "camera_frame";
    EE_pose.header.stamp = ros::Time::now();
    EE_pose.pose.position.x = 1;
    EE_pose.pose.position.y = 2;
    EE_pose.pose.position.z = 3;
    EE_pose.pose.orientation.x = 0;
    EE_pose.pose.orientation.y = 0;
    EE_pose.pose.orientation.z = 0;
    EE_pose.pose.orientation.w = 1;
    Eigen::Translation3d translation(EE_pose.pose.position.x, EE_pose.pose.position.y, EE_pose.pose.position.z);
    Eigen::Quaterniond rotation(EE_pose.pose.orientation.w, EE_pose.pose.orientation.x, EE_pose.pose.orientation.y, EE_pose.pose.orientation.z);
    Eigen::Affine3d target_pose = translation * rotation;

    //srv.request.obj_pose = EE_pose;
    srv.request.object = "Crayons";
    srv.request.obj_pose = EE_pose;

    if (client.call(srv))
     {
       //ROS_INFO("Target Arm Pose", srv.response.arm_pose);
        std::cout << srv.response.arm_pose << std::endl;
     }
     else
     {
       ROS_ERROR("Failed to call service grasp_logic");
       return 1;
     }
    ros::spin();

    return 0;
}
