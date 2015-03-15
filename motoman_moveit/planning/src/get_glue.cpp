#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <motoman_moveit/move_group_server.h>
#include <apc_vision/ObjectDetect.h>
#include <grasp_logic/grasp.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "right_target_pose");
    ros::NodeHandle node_handle;  
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    // move to pose in front of bin
    motoman_moveit::move_group_server moveBin;    

    geometry_msgs::PoseStamped target_pose_stamped;    
    geometry_msgs::Pose target_pose;
    target_pose.position.x = ;
    target_pose.position.y = ;
    target_pose.position.z = ;
    target_pose.orientation.x = ;
    target_pose.orientation.y = ;
    target_pose.orientation.z = ;
    target_pose.orientation.w = ;

    target_pose_stamped.pose = target_pose;

    moveBin.request.pose = target_pose_stamped;
    moveBin.request.arm = "arm_left";

    ros::service::call("move_group_service", moveBin);
    

    // call vision service
    apc_vision::ObjectDetect detect;

    detect.request.bin = "D";
    detect.request.object = "elmers_washable_no_run_school_glue";

    ros::service::call("object_detect", detect);

    // call grasp logic service
    grasp_logic::grasp grasp;

    grasp.request.object = "elmers_washable_no_run_school_glue";
    grasp.request.obj_pose = detect.response.pose;

    ros::service::call("grasp_logic", grasp);

    // move to object    
    motoman_moveit::move_group_server moveObject;   

    moveObject.request.pose = grasp.response.arm_pose;
    moveObject.request.arm = "arm_left";

    ros::service::call("move_group_service", moveObject);


    // call gripper (close) service
    // ADD LATER

    // move up
    motoman_moveit::move_group_server moveUp;   

    moveUp.request.pose = grasp.response.arm_pose;
    moveUp.request.pose.pose.position.z += 0.02;    
    moveUp.request.arm = "arm_left";

    ros::service::call("move_group_service", moveUp);


    // move out (to same pose in front of bin)
    ros::service::call("move_group_service", moveBin);

    // move to order box
    motoman_moveit::move_group_server moveBox;   

    geometry_msgs::PoseStamped box_pose_stamped;    
    geometry_msgs::Pose box_pose;
    box_pose.position.x = ;
    box_pose.position.y = ;
    box_pose.position.z = ;
    box_pose.orientation.x = ;
    box_pose.orientation.y = ;
    box_pose.orientation.z = ;
    box_pose.orientation.w = ;

    box_pose_stamped.pose = box_pose;

    moveBox.request.pose = box_pose_stamped;
    moveBox.request.arm = "arm_left";

    ros::service::call("move_group_service", moveBox);


    // call gripper (open) service
    // ADD LATER


}
