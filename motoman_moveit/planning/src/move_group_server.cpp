#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>

#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <cmath>

#include <motoman_moveit/move_group_server.h>

// currently this function takes a Pose and tells the right end-effector to move there
bool move_callback(motoman_moveit::move_group_server::Request &req,
    motoman_moveit::move_group_server::Response &res)
{
    std::cout << "PLANNING..." << std::endl;
    moveit::planning_interface::MoveGroup move_group(req.arm);
    move_group.setPlannerId("RRTstarkConfigDefault");
    move_group.setPlanningTime(5.0);
    move_group.setStartStateToCurrentState();

    if (req.tolerance) {
        move_group.setGoalOrientationTolerance(0.001); //0.1
        move_group.setGoalPositionTolerance(0.0001);   //0.02
        move_group.setGoalJointTolerance(0.001);       //0.1
    }

    move_group.setPoseTarget(req.pose.pose);

    // compute and visualize plan
    moveit::planning_interface::MoveGroup::Plan plan;
    //moveit::planning_interface::MoveGroup::Plan plan2;


    bool planSuccess = move_group.plan(plan);

    /*
    if(!move_group.plan(plan)) {
            move_group.setPlanningTime(15.0);
            move_group.plan(plan);
    }
    move_group.setPlanningTime(5.0);
    */

    /*
    // ALTERNATE PLAN
    int planSize = plan.trajectory_.joint_trajectory.points.size();
    int n = 2;
    int newSize = planSize/n;
    if (planSize % n != 0) {
        newSize += 1;
    }

    std::vector<trajectory_msgs::JointTrajectoryPoint> newPoints(newSize);
    for (int i = 0; i < newSize - 1; i++) {
        newPoints[i] = plan.trajectory_.joint_trajectory.points[n*i];
    }
    newPoints[newSize - 1] = plan.trajectory_.joint_trajectory.points[planSize - 1];

    plan2.planning_time_ = plan.planning_time_;
    plan2.start_state_.joint_state.header = plan.start_state_.joint_state.header;
    plan2.start_state_.is_diff = true;
    int arraySize = plan.start_state_.joint_state.name.size();
    for (int k = 0; k < arraySize; k++) {
        plan2.start_state_.joint_state.name.push_back(plan.start_state_.joint_state.name[k]);
        plan2.start_state_.joint_state.position.push_back(plan.start_state_.joint_state.position[k]);
        plan2.start_state_.joint_state.velocity.push_back(plan.start_state_.joint_state.velocity[k]);
        plan2.start_state_.joint_state.effort.push_back(plan.start_state_.joint_state.effort[k]);
    }
    for (int j = 0; j < newSize; j++) {
        plan2.trajectory_.joint_trajectory.points.push_back(newPoints[j]);
    }*/

    /*
    // ALTERNATE PLAN
    int planSize = plan.trajectory_.joint_trajectory.points.size();
    trajectory_msgs::JointTrajectoryPoint trajPoint1 = plan.trajectory_.joint_trajectory.points[planSize - 1];

    //plan.trajectory_.joint_trajectory.points[0].time_from_start += ros::Duration(1.0);
    for (int i = 1; i < planSize; i++) {
        plan.trajectory_.joint_trajectory.points[i] = trajPoint1;
        plan.trajectory_.joint_trajectory.points[i].time_from_start += ros::Duration(0.01*(i-1));
        plan.trajectory_.joint_trajectory.points[i].time_from_start *= 3.0; //3.0 WORKED
        for (int j = 0; j < 7; j++) {
            plan.trajectory_.joint_trajectory.points[i].velocities[j] = 0.0;
        }
        std::cout << "Point " << i << ": " << plan.trajectory_.joint_trajectory.points[i].time_from_start << std::endl;
    }*/


    /*int planSize = plan.trajectory_.joint_trajectory.points.size();
    for (int i = 0; i < planSize; i++) {
        plan.trajectory_.joint_trajectory.points[i].time_from_start *= 4.0;
        for (int j = 0; j < 7; j++) {
            plan.trajectory_.joint_trajectory.points[i].velocities[j] *= 0.25;
        }
    }*/

    std::cout << "VISUALIZING..." << std::endl;

    // Sleep while plan is shown in Rviz
    //sleep(3.0);

    std::cout << "MOVING..." << std::endl;

    // move to target pose
    res.success = move_group.execute(plan);
    //res.success = move_group.execute(plan2);

    sleep(2.0);

    std::cout << "EXECUTED!" << std::endl;

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_group_server");
    ros::NodeHandle node_handle;

    ros::ServiceServer service = node_handle.advertiseService("move_group_service", move_callback);
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}
