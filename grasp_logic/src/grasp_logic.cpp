#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include <iostream>

#include "grasp_logic/grasp.h"

bool service_cb(grasp_logic::grasp::Request  &req,
         grasp_logic::grasp::Response &res)
 {
    geometry_msgs::PoseStamped cam_pose;
    Eigen::Affine3d Tpalm_objframe;
    std::string object;
    float offset_x = 0;
    float offset_y = 0;
    float offset_z = 0;
    float palm_offset = 0.0525; // offset from EE to palm on the hand
    float finger_offset = 0.0705; // offset from the palm to hand
    float wrist_offset = 0.08; // offset from the wrist to palm
    float total_offset = palm_offset+finger_offset+wrist_offset; // offset from the pam to hand


    // Extract target object name
    object = req.object;

    // Offsets wrt to robot base frame
    if (object == "elmers_washable_no_run_school_glue")
    {
        offset_x = total_offset;
        offset_y = 0;
        offset_z = -0.005;
    }
    else if (object == "stir_sticks")
    {
        offset_x = total_offset-0.005;
        offset_y = 0;
        offset_z = 0;
    }
    else if (object == "cheezit")
    {
        offset_x = total_offset-0.005;
        offset_y = 0;
        offset_z = 0;
    }
    else if (object == "expo_dry_erase_board_eraser")
    {
        offset_x = total_offset;
        offset_y = 0;
        offset_z = -0.005;
    }
    else if (object == "colored_eggs")
    {
        offset_x = total_offset;
        offset_y = 0;
        offset_z = 0;
    }

    // Extract pose of object from passed in variable req.obj_pose
    cam_pose.header.frame_id = req.obj_pose.header.frame_id;
    cam_pose.header.stamp = req.obj_pose.header.stamp;
    cam_pose.pose.position.x = req.obj_pose.pose.position.x-offset_x;
    cam_pose.pose.position.y = req.obj_pose.pose.position.y-offset_y;
    cam_pose.pose.position.z = req.obj_pose.pose.position.z-offset_z;
    cam_pose.pose.orientation.x = req.obj_pose.pose.orientation.x;
    cam_pose.pose.orientation.y = req.obj_pose.pose.orientation.y;
    cam_pose.pose.orientation.z = req.obj_pose.pose.orientation.z;
    cam_pose.pose.orientation.w = req.obj_pose.pose.orientation.w;

    // Transform from palm of gripper to
    Eigen::Translation3d translation(cam_pose.pose.position.x, cam_pose.pose.position.y, cam_pose.pose.position.z);
    Eigen::Quaterniond rotation(cam_pose.pose.orientation.w, cam_pose.pose.orientation.x, cam_pose.pose.orientation.y, cam_pose.pose.orientation.z);
    Eigen::Affine3d target_pose = translation * rotation;

    // set output for arm_pose
    res.arm_pose = cam_pose;

    // set output for object
    res.object = object;


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
