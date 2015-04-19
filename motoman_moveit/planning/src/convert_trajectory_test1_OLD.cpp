#include <ros/ros.h>
#include <iostream>
#include <moveit_msgs/RobotTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <motoman_moveit/convert_trajectory_server.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "convert_trajectory_test1");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::service::waitForService("convert_trajectory_service");
    std::cout << "SERVICES STARTED!" << std::endl;
    for (int i = 0; i < 10; i++) {
        sleep(1.0);
        std::cout << i+1 << " seconds" << std::endl;
    }
    std::cout << "-------DONE SLEEPING----" << std::endl;

    // create trajectories
    moveit_msgs::RobotTrajectory leftRobot;
    moveit_msgs::RobotTrajectory rightRobot;
    moveit_msgs::RobotTrajectory torsoRobot;

    trajectory_msgs::JointTrajectory leftTraj;
    trajectory_msgs::JointTrajectory rightTraj;
    trajectory_msgs::JointTrajectory torsoTraj;

    std::vector<trajectory_msgs::JointTrajectoryPoint> leftPoints(2);
    std::vector<trajectory_msgs::JointTrajectoryPoint> rightPoints(2);
    std::vector<trajectory_msgs::JointTrajectoryPoint> torsoPoints(2);

    std::vector<std::string> leftJointNames(7);
    leftJointNames[0] = "arm_left_joint_1_s";
    leftJointNames[1] = "arm_left_joint_2_l";
    leftJointNames[2] = "arm_left_joint_3_e";
    leftJointNames[3] = "arm_left_joint_4_u";
    leftJointNames[4] = "arm_left_joint_5_r";
    leftJointNames[5] = "arm_left_joint_6_b";
    leftJointNames[6] = "arm_left_joint_7_t";

    std::vector<std::string> rightJointNames(7);
    rightJointNames[0] = "arm_right_joint_1_s";
    rightJointNames[1] = "arm_right_joint_2_l";
    rightJointNames[2] = "arm_right_joint_3_e";
    rightJointNames[3] = "arm_right_joint_4_u";
    rightJointNames[4] = "arm_right_joint_5_r";
    rightJointNames[5] = "arm_right_joint_6_b";
    rightJointNames[6] = "arm_right_joint_7_t";

    std::vector<std::string> torsoJointNames(1);
    torsoJointNames[0] = "torso_joint_b1";

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


    leftPoints[0].positions = leftPosVector0;
    leftPoints[0].velocities = leftVelVector;
    leftPoints[0].time_from_start = ros::Duration(0.0);
    leftPoints[1].positions = leftPosVector1;
    leftPoints[1].velocities = leftVelVector;
    leftPoints[1].time_from_start = ros::Duration(10.0);

    rightPoints[0].positions = rightPosVector0;
    rightPoints[0].velocities = rightVelVector;
    rightPoints[0].time_from_start = ros::Duration(0.0);
    rightPoints[1].positions = rightPosVector1;
    rightPoints[1].velocities = rightVelVector;
    rightPoints[1].time_from_start = ros::Duration(15.0);

    torsoPoints[0].positions = torsoPosVector0;
    torsoPoints[0].velocities = torsoVelVector;
    torsoPoints[0].time_from_start = ros::Duration(0.0);
    torsoPoints[1].positions = torsoPosVector1;
    torsoPoints[1].velocities = torsoVelVector;
    torsoPoints[1].time_from_start = ros::Duration(5.0);


    leftTraj.header.frame_id = "/base_link";
    leftTraj.joint_names = leftJointNames;
    leftTraj.points = leftPoints;

    rightTraj.header.frame_id = "/base_link";
    rightTraj.joint_names = rightJointNames;
    rightTraj.points = rightPoints;

    torsoTraj.header.frame_id = "/base_link";
    torsoTraj.joint_names = torsoJointNames;
    torsoTraj.points = torsoPoints;

    leftRobot.joint_trajectory = leftTraj;
    rightRobot.joint_trajectory = rightTraj;
    torsoRobot.joint_trajectory = torsoTraj;

    // CALL SERVICE
    motoman_moveit::convert_trajectory_server move1;

    move1.request.leftTraj = leftRobot;
    move1.request.rightTraj = rightRobot;
    move1.request.torsoTraj = torsoRobot;
    move1.request.moveLeft = false;
    move1.request.moveRight = true;
    move1.request.moveTorso = false;

    ros::service::call("convert_trajectory_service", move1);

    if (!move1.response.success) {
        std::cout << "convert_trajectory_service returned failure" << "/n";
    }
    return 0;
}
