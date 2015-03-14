#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include <iostream>

#include "grasp_logic/grasp.h"

bool service_cb(grasp_logic::grasp::Request  &req,
         grasp_logic::grasp::Response &res)
 {
    geometry_msgs::PoseStamped obj_pose;
    Eigen::Affine3d desired_pose;
/*
    // Stamped Pose msg
    obj_pose.header.frame_id = "";
    obj_pose.header.stamp = ros::Time::now();

    obj_pose.pose.position.x = 0.94;
    obj_pose.pose.position.y = -0.31;
    obj_pose.pose.position.z = 0.13;
    obj_pose.pose.orientation.x = 0.3;
    obj_pose.pose.orientation.y = 0.7;
    obj_pose.pose.orientation.z = -0.3;
    obj_pose.pose.orientation.w = 0.7;
    Eigen::Translation3d translation(obj_pose.pose.position.x, obj_pose.pose.position.y, obj_pose.pose.position.z);
    Eigen::Quaterniond rotation(obj_pose.pose.orientation.w, obj_pose.pose.orientation.x, obj_pose.pose.orientation.y, obj_pose.pose.orientation.z);
    Eigen::Affine3d target_pose = translation * rotation;
*/
    return true;
 }


int main(int argc, char** argv)
{
    ros::init(argc, argv, "grasp_logic_service");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("grasp_logic", service_cb);

    ros::spin();

    return 0;
}
