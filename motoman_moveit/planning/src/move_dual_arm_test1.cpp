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
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/planning_scene/planning_scene.h>

#include <motoman_msgs/DynamicJointTrajectory.h>
#include <motoman_msgs/DynamicJointPoint.h>
#include <motoman_msgs/DynamicJointsGroup.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_dual_arm_test1");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Publisher completePub = node_handle.advertise<motoman_msgs::DynamicJointTrajectory>("/joint_path_command", 1);

    ros::service::waitForService("move_group_service");
    std::cout << "SERVICES STARTED!" << std::endl;
    for (int i = 0; i < 10; i++) {
        sleep(1.0);
        std::cout << i+1 << " seconds" << std::endl;
    }
    std::cout << "-------DONE SLEEPING----" << std::endl;

    // move to pose 0
//        moveit::planning_interface::MoveGroup move_group("sda10f");

    // POSITION[0] VECTORS
    std::vector<double> torsoPosVector0(1);
    torsoPosVector0[0] = 0.0;

    std::vector<double> rightPosVector0(7);
    rightPosVector0[0] = 0.0;
    rightPosVector0[1] = 0.0;
    rightPosVector0[2] = 0.0;
    rightPosVector0[3] = 0.0;
    rightPosVector0[4] = 0.0;
    rightPosVector0[5] = 0.0;
    rightPosVector0[6] = 0.0;

    std::vector<double> leftPosVector0(7);
    leftPosVector0[0] = 0.0;
    leftPosVector0[1] = 0.0;
    leftPosVector0[2] = 0.0;
    leftPosVector0[3] = 0.0;
    leftPosVector0[4] = 0.0;
    leftPosVector0[5] = 0.0;
    leftPosVector0[6] = 0.0;

    // POSITION[1] VECTORS
    std::vector<double> torsoPosVector1(1);
    torsoPosVector1[0] = 0.2;

    std::vector<double> rightPosVector1(7);
    rightPosVector1[0] = -1.2;
    rightPosVector1[1] = 1.0;
    rightPosVector1[2] = -1.0;
    rightPosVector1[3] = 1.0;
    rightPosVector1[4] = -1.0;
    rightPosVector1[5] = 1.0;
    rightPosVector1[6] = -1.0;

    std::vector<double> leftPosVector1(7);
    leftPosVector1[0] = -2.0;
    leftPosVector1[1] = 1.0;
    leftPosVector1[2] = -1.0;
    leftPosVector1[3] = 1.0;
    leftPosVector1[4] = -1.0;
    leftPosVector1[5] = 1.0;
    leftPosVector1[6] = -1.0;

    // VELOCITY VECTORS
    std::vector<double> torsoVelVector(1);
    torsoVelVector[0] = 0.0;

    std::vector<double> rightVelVector(7);
    rightVelVector[0] = 0.0;
    rightVelVector[1] = 0.0;
    rightVelVector[2] = 0.0;
    rightVelVector[3] = 0.0;
    rightVelVector[4] = 0.0;
    rightVelVector[5] = 0.0;
    rightVelVector[6] = 0.0;

    std::vector<double> leftVelVector(7);
    leftVelVector[0] = 0.0;
    leftVelVector[1] = 0.0;
    leftVelVector[2] = 0.0;
    leftVelVector[3] = 0.0;
    leftVelVector[4] = 0.0;
    leftVelVector[5] = 0.0;
    leftVelVector[6] = 0.0;

    double trajDuration = 10.0;

    motoman_msgs::DynamicJointsGroup torsoGroups0;
    torsoGroups0.group_number = 2;
    torsoGroups0.valid_fields = 0;
    torsoGroups0.positions = torsoPosVector0;
    torsoGroups0.velocities = torsoVelVector;
    torsoGroups0.time_from_start = ros::Duration(0.0);

    motoman_msgs::DynamicJointsGroup torsoGroups1;
    torsoGroups1.group_number = 2;
    torsoGroups1.valid_fields = 0;
    torsoGroups1.positions = torsoPosVector1;
    torsoGroups1.velocities = torsoVelVector;
    torsoGroups1.time_from_start = ros::Duration(trajDuration);

    motoman_msgs::DynamicJointsGroup torso2Groups0;
    torso2Groups0.group_number = 3;
    torso2Groups0.valid_fields = 0;
    torso2Groups0.positions = torsoPosVector0;
    torso2Groups0.velocities = torsoVelVector;
    torso2Groups0.time_from_start = ros::Duration(0.0);

    motoman_msgs::DynamicJointsGroup torso2Groups1;
    torso2Groups1.group_number = 3;
    torso2Groups1.valid_fields = 0;
    torso2Groups1.positions = torsoPosVector1;
    torso2Groups1.velocities = torsoVelVector;
    torso2Groups1.time_from_start = ros::Duration(trajDuration);

    motoman_msgs::DynamicJointsGroup rightGroups0;
    rightGroups0.group_number = 1;
    rightGroups0.valid_fields = 0;
    rightGroups0.positions = rightPosVector0;
    rightGroups0.velocities = rightVelVector;
    rightGroups0.time_from_start = ros::Duration(0.0);

    motoman_msgs::DynamicJointsGroup rightGroups1;
    rightGroups1.group_number = 1;
    rightGroups1.valid_fields = 0;
    rightGroups1.positions = rightPosVector1;
    rightGroups1.velocities = rightVelVector;
    rightGroups1.time_from_start = ros::Duration(trajDuration);

    motoman_msgs::DynamicJointsGroup leftGroups0;
    leftGroups0.group_number = 0;
    leftGroups0.valid_fields = 0;
    leftGroups0.positions = leftPosVector0;
    leftGroups0.velocities = leftVelVector;
    leftGroups0.time_from_start = ros::Duration(0.0);

    motoman_msgs::DynamicJointsGroup leftGroups1;
    leftGroups1.group_number = 0;
    leftGroups1.valid_fields = 0;
    leftGroups1.positions = leftPosVector1;
    leftGroups1.velocities = leftVelVector;
    leftGroups1.time_from_start = ros::Duration(trajDuration);

    std::vector<std::string> completeJointNames(16);
    completeJointNames[0] = "arm_left_joint_1_s";
    completeJointNames[1] = "arm_left_joint_2_l";
    completeJointNames[2] = "arm_left_joint_3_e";
    completeJointNames[3] = "arm_left_joint_4_u";
    completeJointNames[4] = "arm_left_joint_5_r";
    completeJointNames[5] = "arm_left_joint_6_b";
    completeJointNames[6] = "arm_left_joint_7_t";
    completeJointNames[7] = "arm_right_joint_1_s";
    completeJointNames[8] = "arm_right_joint_2_l";
    completeJointNames[9] = "arm_right_joint_3_e";
    completeJointNames[10] = "arm_right_joint_4_u";
    completeJointNames[11] = "arm_right_joint_5_r";
    completeJointNames[12] = "arm_right_joint_6_b";
    completeJointNames[13] = "arm_right_joint_7_t";
    completeJointNames[14] = "torso_joint_b1";
    completeJointNames[15] = "torso_joint_b2";

    std::vector<motoman_msgs::DynamicJointsGroup> groups0(4);
    groups0[0] = leftGroups0;
    groups0[1] = rightGroups0;
    groups0[2] = torsoGroups0;
    groups0[3] = torso2Groups0;

    std::vector<motoman_msgs::DynamicJointsGroup> groups1(4);
    groups1[0] = leftGroups1;
    groups1[1] = rightGroups1;
    groups1[2] = torsoGroups1;
    groups1[3] = torso2Groups1;

    std::vector<motoman_msgs::DynamicJointPoint> completePoints(2);
    completePoints[0].num_groups = 4;
    completePoints[0].groups = groups0;
    completePoints[1].num_groups = 4;
    completePoints[1].groups = groups1;

    motoman_msgs::DynamicJointTrajectory completeTraj;
    completeTraj.header.frame_id = "/base_link";
    completeTraj.joint_names = completeJointNames;
    completeTraj.points = completePoints;

    completePub.publish(completeTraj);

    for (int i = 0; i < (int)trajDuration; i++) {
        sleep(1.0);
        std::cout << i+1 << " seconds" << std::endl;
    }

    std::cout << "##########   MOVED TO POSE #0   ##########" << std::endl;
    std::cout << "##########   DONE   ##########" << std::endl;
    return true;

}
