#include <ros/ros.h>
#include <iostream>
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
    trajectory_msgs::JointTrajectory jointTraj;

    std::vector<trajectory_msgs::JointTrajectoryPoint> trajPoints(2);

    std::vector<std::string> jointNames(15);
    jointNames[0] = "arm_left_joint_1_s";
    jointNames[1] = "arm_left_joint_2_l";
    jointNames[2] = "arm_left_joint_3_e";
    jointNames[3] = "arm_left_joint_4_u";
    jointNames[4] = "arm_left_joint_5_r";
    jointNames[5] = "arm_left_joint_6_b";
    jointNames[6] = "arm_left_joint_7_t";
    jointNames[7] = "torso_joint_b1";
    jointNames[8] = "arm_right_joint_1_s";
    jointNames[9] = "arm_right_joint_2_l";
    jointNames[10] = "arm_right_joint_3_e";
    jointNames[11] = "arm_right_joint_4_u";
    jointNames[12] = "arm_right_joint_5_r";
    jointNames[13] = "arm_right_joint_6_b";
    jointNames[14] = "arm_right_joint_7_t";


    // POSITION[0] VECTOR
    std::vector<double> posVector0(15);
    posVector0[0] = 0.0;
    posVector0[1] = 0.0;
    posVector0[2] = 0.0;
    posVector0[3] = 0.0;
    posVector0[4] = 0.0;
    posVector0[5] = 0.0;
    posVector0[6] = 0.0;
    posVector0[7] = 0.0;
    posVector0[8] = 0.0;
    posVector0[9] = 0.0;
    posVector0[10] = 0.0;
    posVector0[11] = 0.0;
    posVector0[12] = 0.0;
    posVector0[13] = 0.0;
    posVector0[14] = 0.0;

    // POSITION[1] VECTOR
    std::vector<double> posVector1(15);
    posVector1[0] = -1.0;
    posVector1[1] = 1.0;
    posVector1[2] = -1.0;
    posVector1[3] = 1.0;
    posVector1[4] = -1.0;
    posVector1[5] = 1.0;
    posVector1[6] = -1.0;
    posVector1[7] = 0.3;
    posVector1[8] = -1.0;
    posVector1[9] = 1.0;
    posVector1[10] = -1.0;
    posVector1[11] = 1.0;
    posVector1[12] = -1.0;
    posVector1[13] = 1.0;
    posVector1[14] = -1.0;

    // VELOCITY VECTOR
    std::vector<double> velVector(15);
    velVector[0] = 0.0;
    velVector[1] = 0.0;
    velVector[2] = 0.0;
    velVector[3] = 0.0;
    velVector[4] = 0.0;
    velVector[5] = 0.0;
    velVector[6] = 0.0;
    velVector[7] = 0.0;
    velVector[8] = 0.0;
    velVector[9] = 0.0;
    velVector[10] = 0.0;
    velVector[11] = 0.0;
    velVector[12] = 0.0;
    velVector[13] = 0.0;
    velVector[14] = 0.0;

    trajPoints[0].positions = posVector0;
    trajPoints[0].velocities = velVector;
    trajPoints[0].time_from_start = ros::Duration(0.0);
    trajPoints[1].positions = posVector1;
    trajPoints[1].velocities = velVector;
    trajPoints[1].time_from_start = ros::Duration(10.0);

    jointTraj.header.frame_id = "/base_link";
    jointTraj.joint_names = jointNames;
    jointTraj.points = trajPoints;

    // CALL SERVICE
    motoman_moveit::convert_trajectory_server move1;

    move1.request.jointTraj = jointTraj;

    ros::service::call("convert_trajectory_service", move1);

    if (!move1.response.success) {
        std::cout << "convert_trajectory_service returned failure" << "/n";
    }
    return 0;
}
