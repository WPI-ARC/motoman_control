#include <ros/ros.h>
#include <iostream>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <motoman_msgs/DynamicJointTrajectory.h>
#include <motoman_msgs/DynamicJointPoint.h>
#include <motoman_msgs/DynamicJointsGroup.h>

#include <motoman_moveit/convert_trajectory_server.h>

ros::Publisher completePub;

bool move_callback(motoman_moveit::convert_trajectory_server::Request &req,
    motoman_moveit::convert_trajectory_server::Response &res) {

    std::vector<std::string> jointNames;

    std::vector<motoman_msgs::DynamicJointPoint> trajPoints;

    std::vector<motoman_msgs::DynamicJointsGroup> leftGroupPoints;
    std::vector<motoman_msgs::DynamicJointsGroup> rightGroupPoints;
    std::vector<motoman_msgs::DynamicJointsGroup> torso1GroupPoints;
    std::vector<motoman_msgs::DynamicJointsGroup> torso2GroupPoints;

    trajectory_msgs::JointTrajectory leftTraj;
    trajectory_msgs::JointTrajectory rightTraj;
    trajectory_msgs::JointTrajectory torsoTraj;

    int leftSize;
    int rightSize;
    int torsoSize;


    int i = 0;

    if(req.moveLeft) {
        leftTraj = req.leftTraj.joint_trajectory;
        for(i = 0; i < 7; i++) {
            jointNames.push_back(leftTraj.joint_names[i]);
        }
        leftSize = leftTraj.points.size();
        if(leftSize > trajPoints.size()) {
            trajPoints.resize(leftSize);
        }
        leftGroupPoints.resize(leftSize);
        for(i = 0; i < leftSize; i++) {
            leftGroupPoints[i].group_number = 0;
            leftGroupPoints[i].num_joints = 7;
            leftGroupPoints[i].positions = leftTraj.points[i].positions;
            leftGroupPoints[i].velocities = leftTraj.points[i].velocities;
            leftGroupPoints[i].time_from_start = leftTraj.points[i].time_from_start;
        }
    }

    if(req.moveRight) {
        rightTraj = req.rightTraj.joint_trajectory;
        for(i = 0; i < 7; i++) {
            jointNames.push_back(rightTraj.joint_names[i]);
        }
        rightSize = rightTraj.points.size();
        if(rightSize > trajPoints.size()) {
            trajPoints.resize(rightSize);
        }
        rightGroupPoints.resize(rightSize);
        for(i = 0; i < rightSize; i++) {
            rightGroupPoints[i].group_number = 1;
            rightGroupPoints[i].num_joints = 7;
            rightGroupPoints[i].positions = rightTraj.points[i].positions;
            rightGroupPoints[i].velocities = rightTraj.points[i].velocities;
            rightGroupPoints[i].time_from_start = rightTraj.points[i].time_from_start;
        }

    }

    if(req.moveTorso) {
        torsoTraj = req.torsoTraj.joint_trajectory;
        jointNames.push_back("torso_joint_b1");
        jointNames.push_back("torso_joint_b2");
        torsoSize = torsoTraj.points.size();
        if(torsoSize > trajPoints.size()) {
            trajPoints.resize(torsoSize);
        }
        torso1GroupPoints.resize(torsoSize);
        torso2GroupPoints.resize(torsoSize);
        for(i = 0; i < torsoSize; i++) {
            torso1GroupPoints[i].group_number = 2;
            torso1GroupPoints[i].num_joints = 1;
            torso1GroupPoints[i].positions = torsoTraj.points[i].positions;
            torso1GroupPoints[i].velocities = torsoTraj.points[i].velocities;
            torso1GroupPoints[i].time_from_start = torsoTraj.points[i].time_from_start;

            torso2GroupPoints[i].group_number = 3;
            torso2GroupPoints[i].num_joints = 1;
            torso2GroupPoints[i].positions = torsoTraj.points[i].positions;
            torso2GroupPoints[i].velocities = torsoTraj.points[i].velocities;
            torso2GroupPoints[i].time_from_start = torsoTraj.points[i].time_from_start;
        }

    }

    int num_groups;
    std::vector<motoman_msgs::DynamicJointsGroup> groups;

    for(i = 0; i < trajPoints.size(); i++) {
        num_groups = 0;
        groups.clear();
        if(req.moveLeft) {
            if(leftSize >= i) {
                num_groups += 1;
                groups.push_back(leftGroupPoints[i]);
            }
        }
        if(req.moveRight) {
            if(rightSize >= i) {
                num_groups += 1;
                groups.push_back(rightGroupPoints[i]);
            }
        }
        if(req.moveTorso) {
            if(torsoSize >= i) {
                num_groups += 2;
                groups.push_back(torso1GroupPoints[i]);
                groups.push_back(torso2GroupPoints[i]);
            }
        }
        trajPoints[i].num_groups = num_groups;
        trajPoints[i].groups = groups;
    }

    motoman_msgs::DynamicJointTrajectory completeTraj;
    completeTraj.header.frame_id = "/base_link";
    completeTraj.joint_names = jointNames;
    completeTraj.points = trajPoints;

    completePub.publish(completeTraj);

    double trajDuration = 0;
    if(req.moveLeft) {
        if(leftTraj.points[leftSize-1].time_from_start.toSec() > trajDuration) {
            trajDuration = leftTraj.points[leftSize-1].time_from_start.toSec();
        }
    }

    if(req.moveRight) {
        if(rightTraj.points[rightSize-1].time_from_start.toSec() > trajDuration) {
            trajDuration = rightTraj.points[rightSize-1].time_from_start.toSec();
        }
    }

    if(req.moveTorso) {
        if(torsoTraj.points[torsoSize-1].time_from_start.toSec() > trajDuration) {
            trajDuration = torsoTraj.points[torsoSize-1].time_from_start.toSec();
        }
    }

    for (int i = 0; i < (int)trajDuration + 1; i++) {
        sleep(1.0);
        std::cout << i+1 << " seconds" << std::endl;
    }

    std::cout << "##########   MOVED TO POSE #0   ##########" << std::endl;
    std::cout << "##########   DONE   ##########" << std::endl;
    res.success = true;
    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "convert_trajectory_server");
    ros::NodeHandle node_handle;

    completePub = node_handle.advertise<motoman_msgs::DynamicJointTrajectory>("/joint_path_command", 1);

    ros::ServiceServer service = node_handle.advertiseService("convert_trajectory_service", move_callback);
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}
