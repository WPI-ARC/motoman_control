#include <ros/ros.h>
#include <iostream>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <motoman_msgs/DynamicJointTrajectory.h>
#include <motoman_msgs/DynamicJointPoint.h>
#include <motoman_msgs/DynamicJointsGroup.h>
#include <sensor_msgs/JointState.h>

#include <motoman_moveit/convert_trajectory_server.h>

ros::Publisher completePub;

std::vector<sensor_msgs::JointState> jointStates(4);

bool move_callback(motoman_moveit::convert_trajectory_server::Request &req,
                   motoman_moveit::convert_trajectory_server::Response &res) {

    std::vector<motoman_msgs::DynamicJointPoint> trajPoints;

    std::vector<motoman_msgs::DynamicJointsGroup> leftGroupPoints;
    std::vector<motoman_msgs::DynamicJointsGroup> rightGroupPoints;
    std::vector<motoman_msgs::DynamicJointsGroup> torso1GroupPoints;
    std::vector<motoman_msgs::DynamicJointsGroup> torso2GroupPoints;

    std::vector<double> leftPositions(7);
    std::vector<double> rightPositions(7);
    std::vector<double> torsoPositions(1);

    std::vector<double> leftVelocities(7);
    std::vector<double> rightVelocities(7);
    std::vector<double> torsoVelocities(1);

    trajectory_msgs::JointTrajectory msgTraj;

    int msgSize;
    int msgJointsSize;
    int leftSize;
    int rightSize;
    int torsoSize;
    int num_groups;
    int i;
    int j;

    int leftJointsStart;
    int rightJointsStart;
    int torsoJointsStart;

    // booleans for whether or not particular control groups will be given trajectories
    bool moveLeft = false;
    bool moveRight = false;
    bool moveTorso = false;

    msgTraj = req.jointTraj;
    msgSize = msgTraj.points.size();
    msgJointsSize = msgTraj.joint_names.size();
    trajPoints.resize(msgSize);



//    std::map<std::string, double> joint_commands;
//    for (joint : joints)
//    {
//        joint_commands[joint_name] = joint_value;
//    }
//    // check
//    for (name : left_arm_names)
//    {
//        std::map<std::string, double>::const_iterator found = joint_commands.find(name);
//        if (found != joint_commands.end())
//        {
//            // joint is there
//        }
//        else
//        {
//            // not there
//        }
//    }

    // if joint names for left/right/torso are contained in message joint_names,
    // set corresponding booleans to true
    for(i = 0; i < msgJointsSize; i++) {
        if(msgTraj.joint_names[i] == "arm_left_joint_1_s") {
            moveLeft = true;
            leftJointsStart = i;
        }
        if(msgTraj.joint_names[i] == "arm_right_joint_1_s") {
            moveRight = true;
            rightJointsStart = i;
        }
        if(msgTraj.joint_names[i] == "torso_joint_b1") {
            moveTorso = true;
            torsoJointsStart = i;
        }
    }

    // vectors for all 'zero' velocity values
    std::vector<double> zeroVelArm(7);
    zeroVelArm[0] = 0.0;
    zeroVelArm[1] = 0.0;
    zeroVelArm[2] = 0.0;
    zeroVelArm[3] = 0.0;
    zeroVelArm[4] = 0.0;
    zeroVelArm[5] = 0.0;
    zeroVelArm[6] = 0.0;

    std::vector<double> zeroVelTorso(1);
    zeroVelTorso[0] = 0.0;

    // fill in DynamicPointsGroup vectors
    // create dummy trajectories for any controllers not receiving trajectories
    if(moveLeft) {
        leftSize = msgSize;
        leftGroupPoints.resize(leftSize);
        for(i = 0; i < leftSize; i++) {
            for(j = 0; j < 7; j++) {
                leftPositions[j] = msgTraj.points[i].positions[j + leftJointsStart];
            }
            for(j = 0; j < 7; j++) {
                leftVelocities[j] = msgTraj.points[i].velocities[j + leftJointsStart];
            }
            leftGroupPoints[i].group_number = 0;
            leftGroupPoints[i].num_joints = 7;
            leftGroupPoints[i].positions = leftPositions;
            leftGroupPoints[i].velocities = leftVelocities;
            leftGroupPoints[i].time_from_start = msgTraj.points[i].time_from_start;
        }
    }else {
        leftSize = 2;
        leftGroupPoints.resize(leftSize);
        leftGroupPoints[0].group_number = 0;
        leftGroupPoints[0].num_joints = 7;
        leftGroupPoints[0].positions = jointStates[0].position;
        leftGroupPoints[0].velocities = zeroVelArm;
        leftGroupPoints[0].time_from_start = ros::Duration(0.0);

        leftGroupPoints[1].group_number = 0;
        leftGroupPoints[1].num_joints = 7;
        leftGroupPoints[1].positions = jointStates[0].position;
        leftGroupPoints[1].velocities = zeroVelArm;
        leftGroupPoints[1].time_from_start = ros::Duration(0.1);
    }

    if(moveRight) {
        rightSize = msgSize;
        rightGroupPoints.resize(rightSize);
        for(i = 0; i < rightSize; i++) {
            for(j = 0; j < 7; j++) {
                rightPositions[j] = msgTraj.points[i].positions[j + rightJointsStart];
            }
            for(j = 0; j < 7; j++) {
                rightVelocities[j] = msgTraj.points[i].velocities[j + rightJointsStart];
            }
            rightGroupPoints[i].group_number = 1;
            rightGroupPoints[i].num_joints = 7;
            rightGroupPoints[i].positions = rightPositions;
            rightGroupPoints[i].velocities = rightVelocities;
            rightGroupPoints[i].time_from_start = msgTraj.points[i].time_from_start;
        }
    }else {
        rightSize = 2;
        rightGroupPoints.resize(rightSize);
        rightGroupPoints[0].group_number = 1;
        rightGroupPoints[0].num_joints = 7;
        rightGroupPoints[0].positions = jointStates[1].position;
        rightGroupPoints[0].velocities = zeroVelArm;
        rightGroupPoints[0].time_from_start = ros::Duration(0.0);

        rightGroupPoints[1].group_number = 1;
        rightGroupPoints[1].num_joints = 7;
        rightGroupPoints[1].positions = jointStates[1].position;
        rightGroupPoints[1].velocities = zeroVelArm;
        rightGroupPoints[1].time_from_start = ros::Duration(0.1);
    }

    if(moveTorso) {
        torsoSize = msgSize;
        torso1GroupPoints.resize(torsoSize);
        torso2GroupPoints.resize(torsoSize);
        for(i = 0; i < torsoSize; i++) {
            torsoPositions[0] = msgTraj.points[i].positions[torsoJointsStart];
            torsoVelocities[0] = msgTraj.points[i].velocities[torsoJointsStart];

            torso1GroupPoints[i].group_number = 2;
            torso1GroupPoints[i].num_joints = 1;
            torso1GroupPoints[i].positions = torsoPositions;
            torso1GroupPoints[i].velocities = torsoVelocities;
            torso1GroupPoints[i].time_from_start = msgTraj.points[i].time_from_start;

            torso2GroupPoints[i].group_number = 3;
            torso2GroupPoints[i].num_joints = 1;
            torso2GroupPoints[i].positions = torsoPositions;
            torso2GroupPoints[i].velocities = torsoVelocities;
            torso2GroupPoints[i].time_from_start = msgTraj.points[i].time_from_start;
        }
    }else {
        torsoSize = 2;
        torso1GroupPoints.resize(torsoSize);
        torso1GroupPoints[0].group_number = 2;
        torso1GroupPoints[0].num_joints = 1;
        torso1GroupPoints[0].positions = jointStates[2].position;
        torso1GroupPoints[0].velocities = zeroVelTorso;
        torso1GroupPoints[0].time_from_start = ros::Duration(0.0);

        torso1GroupPoints[1].group_number = 2;
        torso1GroupPoints[1].num_joints = 1;
        torso1GroupPoints[1].positions = jointStates[2].position;
        torso1GroupPoints[1].velocities = zeroVelTorso;
        torso1GroupPoints[1].time_from_start = ros::Duration(0.1);

        torso2GroupPoints.resize(torsoSize);
        torso2GroupPoints[0].group_number = 3;
        torso2GroupPoints[0].num_joints = 1;
        torso2GroupPoints[0].positions = jointStates[3].position;
        torso2GroupPoints[0].velocities = zeroVelTorso;
        torso2GroupPoints[0].time_from_start = ros::Duration(0.0);

        torso2GroupPoints[1].group_number = 3;
        torso2GroupPoints[1].num_joints = 1;
        torso2GroupPoints[1].positions = jointStates[3].position;
        torso2GroupPoints[1].velocities = zeroVelTorso;
        torso2GroupPoints[1].time_from_start = ros::Duration(0.1);
    }

    std::vector<motoman_msgs::DynamicJointsGroup> groups;

    // create final DynamicJointsGroup vector to be published
    for(i = 0; i < msgSize; i++) {
        // for each point, num_groups must match array size
        num_groups = 0;
        groups.clear();
        if(leftSize > i) {
            num_groups += 1;
            groups.push_back(leftGroupPoints[i]);
        }
        if(rightSize > i) {
            num_groups += 1;
            groups.push_back(rightGroupPoints[i]);
        }
        if(torsoSize > i) {
            num_groups += 2;
            groups.push_back(torso1GroupPoints[i]);
            groups.push_back(torso2GroupPoints[i]);
        }
        trajPoints[i].num_groups = num_groups;
        trajPoints[i].groups = groups;
    }


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

    // create final message to be published
    motoman_msgs::DynamicJointTrajectory completeTraj;
    completeTraj.header.frame_id = "/base_link";
    completeTraj.joint_names = completeJointNames;
    completeTraj.points = trajPoints;

    // print trajectory message
    std::cout << "##########   TRAJECTORY:   ##########" << std::endl;
    std::cout << completeTraj << std::endl;

    // publish trrajectory
    completePub.publish(completeTraj);

    double trajDuration = 0;
    trajDuration = msgTraj.points[msgSize-1].time_from_start.toSec();

    // sleep for expected duration of trajectory (rounded up to nearest second)
    for (int i = 0; i < (int)trajDuration + 1; i++) {
        sleep(1.0);
        std::cout << i + 1 << " seconds" << std::endl;
    }

    std::cout << "##########   COMPLETED TRAJECTORY   ##########" << std::endl;
    res.success = true;
    return true;
}

// callback to get joint values from current robot state
void jointStateCallback(sensor_msgs::JointState msg) {
    if(msg.name[0] == "arm_left_joint_1_s") {
        jointStates[0] = msg;
    }
    if(msg.name[0] == "arm_right_joint_1_s") {
        jointStates[1] = msg;
    }
    if(msg.name[0] == "torso_joint_b1") {
        jointStates[2] = msg;
    }
    if(msg.name[0] == "torso_joint_b2") {
        jointStates[3] = msg;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "convert_trajectory_server");
    ros::NodeHandle node_handle;

    completePub = node_handle.advertise<motoman_msgs::DynamicJointTrajectory>("/joint_path_command", 1);

    ros::Subscriber jointStateSub = node_handle.subscribe("/joint_states", 0, jointStateCallback);

    ros::ServiceServer service = node_handle.advertiseService("convert_trajectory_service", move_callback);
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}
