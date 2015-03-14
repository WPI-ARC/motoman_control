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

    // Stamped Pose msg
    obj_pose.header.frame_id = req.obj_pose.header.frame_id;
    obj_pose.header.stamp = ros::Time::now();

    obj_pose.pose.position.x = req.obj_pose.pose.position.x;
    obj_pose.pose.position.y = req.obj_pose.pose.position.y;
    obj_pose.pose.position.z = req.obj_pose.pose.position.z;
    obj_pose.pose.orientation.x = req.obj_pose.pose.orientation.x;
    obj_pose.pose.orientation.y = req.obj_pose.pose.orientation.y;
    obj_pose.pose.orientation.z = req.obj_pose.pose.orientation.z;
    obj_pose.pose.orientation.w = req.obj_pose.pose.orientation.w;
    Eigen::Translation3d translation(obj_pose.pose.position.x, obj_pose.pose.position.y, obj_pose.pose.position.z);
    Eigen::Quaterniond rotation(obj_pose.pose.orientation.w, obj_pose.pose.orientation.x, obj_pose.pose.orientation.y, obj_pose.pose.orientation.z);
    Eigen::Affine3d target_pose = translation * rotation;


    res.arm_pose = obj_pose;

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
