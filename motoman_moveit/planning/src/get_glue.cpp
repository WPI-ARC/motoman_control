#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <motoman_moveit/move_group_server.h>
#include <apc_vision/ObjectDetect.h>
#include <grasp_logic/grasp.h>
#include <iostream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "right_target_pose");
    ros::NodeHandle node_handle;  
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    ros::service::waitForService("move_group_service");
    ros::service::waitForService("object_detect");
    ros::service::waitForService("grasp_logic");
    std::cout << "SERVICES STARTED!" << std::endl;
    sleep(40.0);
    std::cout << "-------DONE SLEEPING----" << std::endl;
    
    // activate gripper
    // ADD LATER
    
    // move to pose in front of bin
    motoman_moveit::move_group_server moveBin;    

    geometry_msgs::PoseStamped target_pose_stamped;    
    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.645954;
    target_pose.position.y = 0.277107;
    target_pose.position.z = 1.41587;
    target_pose.orientation.x = -0.12384;
    target_pose.orientation.y = 0.0841883;
    target_pose.orientation.z = -0.730178;
    target_pose.orientation.w = 0.666646;

    target_pose_stamped.pose = target_pose;

    moveBin.request.pose = target_pose_stamped;
    moveBin.request.arm = "arm_left";

    ros::service::call("move_group_service", moveBin);
    
    if (!moveBin.response.success) {
		std::cout << "move_group_service service returned failure" << "/n";
		return 0;
	}
	std::cout << "##### Moved to bin ######" << std::endl;
    
    // call vision service
    apc_vision::ObjectDetect detect;

    detect.request.bin = "D";
    detect.request.object = "elmers_washable_no_run_school_glue";

    ros::service::call("object_detect", detect);
    
    if (!detect.response.found) {
		std::cout << "object_detect service returned failure" << "/n";
		return 0;
	}

	std::cout << "###### Found object #######" << std::endl;

    // call grasp logic service
    grasp_logic::grasp grasp;

    grasp.request.object = "elmers_washable_no_run_school_glue";
    grasp.request.obj_pose = detect.response.pose;

    ros::service::call("grasp_logic", grasp);

	std::cout << "####### Found grasp #######" << std::endl;
	
    // move to object    
    motoman_moveit::move_group_server moveObject;   

    moveObject.request.pose = grasp.response.arm_pose;
    moveObject.request.arm = "arm_left";

    ros::service::call("move_group_service", moveObject);
    
    if (!moveObject.response.success) {
		std::cout << "move_group_service service returned failure" << "/n";
		return 0;
	}
	
	std::cout << "###### Moved to object #######" << std::endl;


    // call gripper (close) service
    // ADD LATER

    // move up
    motoman_moveit::move_group_server moveUp;   

    moveUp.request.pose = grasp.response.arm_pose;
    moveUp.request.pose.pose.position.z += 0.02;    
    moveUp.request.arm = "arm_left";

    ros::service::call("move_group_service", moveUp);
    
    if (!moveUp.response.success) {
		std::cout << "move_group_service service returned failure" << "/n";
		return 0;
	}
	
	std::cout << "###### Moved up ######" << std::endl;

    // move out (to same pose in front of bin)
    ros::service::call("move_group_service", moveBin);
    
    if (!moveBin.response.success) {
		std::cout << "move_group_service service returned failure" << "/n";
		return 0;
	}
	
	std::cout << "####### Moved to bin ########" << std::endl;

    // move to order box
    motoman_moveit::move_group_server moveBox;   

    geometry_msgs::PoseStamped box_pose_stamped;    
    geometry_msgs::Pose box_pose;
    box_pose.position.x = -0.0245724;
    box_pose.position.y = 0.679758;
    box_pose.position.z = 0.51537;
    box_pose.orientation.x = -0.696986;
    box_pose.orientation.y = -0.225653;
    box_pose.orientation.z = 0.246046;
    box_pose.orientation.w = 0.634628;

    box_pose_stamped.pose = box_pose;

    moveBox.request.pose = box_pose_stamped;
    moveBox.request.arm = "arm_left";

    ros::service::call("move_group_service", moveBox);
    
    if (!moveBox.response.success) {
		std::cout << "move_group_service service returned failure" << "/n";
		return 0;
	}
	
	std::cout << "###### Moved to box ######" << std::endl;


    // call gripper (open) service
    // ADD LATER


}
