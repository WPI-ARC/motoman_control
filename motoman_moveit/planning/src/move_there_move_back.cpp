#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <motoman_moveit/move_group_server.h>
#include <apc_vision/ObjectDetect.h>
#include <grasp_logic/grasp.h>
#include <gripper_srv/gripper.h>
#include <trajectory_srv/task.h>
#include <iostream>
#include <moveit/move_group_interface/move_group.h>

#include <geometry_msgs/Pose.h>
#include <shape_msgs/SolidPrimitive.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene/planning_scene.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_there_move_back");
    ros::NodeHandle node_handle;  
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    ros::service::waitForService("move_group_service");
    std::cout << "SERVICES STARTED!" << std::endl;
    for (int i = 0; i < 15; i++) { 
		sleep(1.0);
		std::cout << i << " seconds" << std::endl;
	}
    std::cout << "-------DONE SLEEPING----" << std::endl;
    
    while (1) {
		// move to pose 1
		motoman_moveit::move_group_server move1;    

		geometry_msgs::PoseStamped target_pose_stamped;    
		geometry_msgs::Pose target_pose;
		target_pose.position.x = 0.429567;
		target_pose.position.y = 0.859446;
		target_pose.position.z = 1.25832;
		target_pose.orientation.x = -0.673667;
		target_pose.orientation.y = 0.207501;
		target_pose.orientation.z = 0.706735;
		target_pose.orientation.w = -0.0603374;

		target_pose_stamped.pose = target_pose;

		move1.request.pose = target_pose_stamped;
		move1.request.arm = "arm_left";
		move1.request.tolerance = true;

		ros::service::call("move_group_service", move1);
		
		if (!move1.response.success) {
			std::cout << "move_group_service service returned failure" << "/n";
			return 0;
		}
		std::cout << "##########   MOVED TO POSE #1   ##########" << std::endl;
		
		// move to pose 2
		motoman_moveit::move_group_server move2;    

		target_pose.position.x = 0.19253;
		target_pose.position.y = 1.11965;
		target_pose.position.z = 1.21749;
		target_pose.orientation.x = 0.706825;
		target_pose.orientation.y = 0.0;
		target_pose.orientation.z = 0.0;
		target_pose.orientation.w = 0.707388;

		target_pose_stamped.pose = target_pose;

		move2.request.pose = target_pose_stamped;
		move2.request.arm = "arm_left";
		move2.request.tolerance = true;

		ros::service::call("move_group_service", move2);
		
		if (!move1.response.success) {
			std::cout << "move_group_service service returned failure" << "/n";
			return 0;
		}
		std::cout << "##########   MOVED TO POSE #2   ##########" << std::endl;
	}

}
