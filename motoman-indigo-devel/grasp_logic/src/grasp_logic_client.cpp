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

    std::string obj_name = req.object;

    // copy posed stamped msg from requestor
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
    Eigen::Affine3d current_pose = translation * rotation;




    return true;
 }


int main(int argc, char** argv)
{
    ros::init(argc, argv, "grasp_logic_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<grasp_logic::grasp>("grasp_logic");
    grasp_logic::grasp srv;

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

    srv.request.obj_pose = target_pose;
    srv.request.object = "Crayons";

    if (client.call(srv))
     {
       ROS_INFO("Target Arm Pose", (long int)srv.response.arm_pose);
     }
     else
     {
       ROS_ERROR("Failed to call service grasp_logic");
       return 1;

    ros::spin();

    return 0;
}
