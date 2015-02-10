#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <iostream>


void right_callback(geometry_msgs::Pose msg)
{
  moveit::planning_interface::MoveGroup group_right("arm_right");
  geometry_msgs::Pose target_pose_right;

  target_pose_right.position.x = msg.position.x;
  target_pose_right.position.y = msg.position.y;
  target_pose_right.position.z = msg.position.z;
  target_pose_right.orientation.x = msg.orientation.x;
  target_pose_right.orientation.y = msg.orientation.y;
  target_pose_right.orientation.z = msg.orientation.z;
  target_pose_right.orientation.w = msg.orientation.w;
  
  group_right.setPoseTarget(target_pose_right);

  // compute and visualize plan
  moveit::planning_interface::MoveGroup::Plan right_plan;
  bool success = group_right.plan(right_plan);

  std::cout << "VISUALIZING..." << "\n";

  // Sleep while plan is shown in Rviz
  sleep(5.0);

  std::cout << "MOVING..." << "\n";

  // move to target pose
  group_right.move();
}

void left_callback(geometry_msgs::Pose msg)
{
  moveit::planning_interface::MoveGroup group_left("arm_left");
  geometry_msgs::Pose target_pose_left;

  target_pose_left.position.x = msg.position.x;
  target_pose_left.position.y = msg.position.y;
  target_pose_left.position.z = msg.position.z;
  target_pose_left.orientation.x = msg.orientation.x;
  target_pose_left.orientation.y = msg.orientation.y;
  target_pose_left.orientation.z = msg.orientation.z;
  target_pose_left.orientation.w = msg.orientation.w;
  
  group_left.setPoseTarget(target_pose_left);

  // compute and visualize plan
  moveit::planning_interface::MoveGroup::Plan left_plan;
  bool success = group_left.plan(left_plan);
 
  // Sleep while plan is shown in Rviz
  sleep(5.0);

  // move to target pose
  group_left.move();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_listen");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(4);
  spinner.start();
  
  // let Rviz start
  sleep(15.0);

  // wait for new target pose for either end-effector
  ros::Subscriber right_sub = node_handle.subscribe("/arm_right_target", 1, right_callback);
  ros::Subscriber left_sub = node_handle.subscribe("/arm_left_target", 1, left_callback);

  std::cout << "WAITING..." << "\n";


  while(1){}

}
