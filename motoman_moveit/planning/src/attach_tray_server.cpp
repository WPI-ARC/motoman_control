#include <ros/ros.h>
#include <iostream>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <motoman_msgs/DynamicJointTrajectory.h>
#include <motoman_msgs/DynamicJointPoint.h>
#include <motoman_msgs/DynamicJointsGroup.h>

#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <motoman_moveit/get_tray_server.h>
#include <motoman_moveit/convert_trajectory_server.h>

#include <ros/ros.h>
#include <mapping_msgs/AttachedCollisionObject.h>
#include <geometric_shapes_msgs/Shape.h>

ros::Publisher att_object_in_map_pub_;
att_object_in_map_pub_  = nh.advertise<mapping_msgs::AttachedCollisionObject>("attached_collision_object", 10);

bool attach_callback(motoman_moveit::attach_tray_server::Request &req,
	    motoman_moveit::attach_tray_server::Response &res) {

	sleep(2);

	//add tray into the collision space attached to the palm
	mapping_msgs::AttachedCollisionObject att_object;
	att_object.link_name = "arm_left_hand_left_palm";
	att_object.touch_links.push_back("arm_left_hand_left_palm");

	att_object.object.id = "attached_tray";
	att_object.object.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
	att_object.object.header.frame_id = "arm_left_hand_left_palm";
	att_object.object.header.stamp = ros::Time::now();
	geometric_shapes_msgs::Shape object;
	object.type = geometric_shapes_msgs::Shape::BOX;
	object.dimensions.resize(3);
	object.dimensions[0] = 0.02;
	object.dimensions[1] = 0.5;
	object.dimensions[2] = 0.3;
	geometry_msgs::Pose pose;
	pose.position.x = 0.0;
	pose.position.y = 0.0;
	pose.position.z = 0.0;
	pose.orientation.x = 0;
	pose.orientation.y = 0;
	pose.orientation.z = 0;
	pose.orientation.w = 1;
	att_object.object.shapes.push_back(object);
	att_object.object.poses.push_back(pose);

	att_object_in_map_pub_.publish(att_object);

	ROS_INFO("Should have published");

	res.success = true;
	return true;


//	ros::Duration(2.0).sleep();

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "attach_tray_server");
    ros::NodeHandle node_handle;

    node_handle.advertiseService("attach_tray_service", attach_callback);
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}
