#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>

#include <motoman_moveit/move_group_server.h>

// currently this function takes a Pose and tells the right end-effector to move there
bool move_callback(motoman_moveit::move_group_server::Request &req,
    motoman_moveit::move_group_server::Request &res)
{
    moveit::planning_interface::MoveGroup move_group(req.arm);
    move_group.setPlannerId("RRTstarkConfigDefault");
	move_group.setPlanningTime(20.0);
	move_group.setStartStateToCurrentState();

    move_group.setPoseTarget(req.pose.pose);

    // compute and visualize plan
    moveit::planning_interface::MoveGroup::Plan plan;
    bool success = move_group.plan(plan);
    
    std::cout << "VISUALIZING..." << "\n";
    
    // Sleep while plan is shown in Rviz
    sleep(5.0);

    std::cout << "MOVING..." << "\n";
    
    // move to target pose
    res.success = move_group.execute(plan);

    return true;
}
    
int main(int argc, char **argv) {
    ros::init(argc, argv, "move_group_server");
    ros::NodeHandle node_handle;
    
    ros::ServiceServer service = node_handle.advertiseService("move_group_service", move_callback);
    ros::spin();

    return 0;
}
