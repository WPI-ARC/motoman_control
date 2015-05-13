#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <motoman_msgs/DynamicJointTrajectory.h>
#include <motoman_msgs/DynamicJointPoint.h>
#include <motoman_msgs/DynamicJointsGroup.h>
#include <motoman_msgs/CmdJointTrajectoryEx.h>
#include <sensor_msgs/JointState.h>
#include <moveit/move_group_interface/move_group.h>

#include <motoman_moveit/convert_trajectory_server.h>

ros::ServiceClient jointPathCommand;

std::vector<sensor_msgs::JointState> jointStates(4);

bool move_callback(motoman_moveit::convert_trajectory_server::Request &req,
                   motoman_moveit::convert_trajectory_server::Response &res) {

    if (req.jointTraj.points.size() == 0) {
        ROS_WARN("Received empty trajectory, not doing anything.");
        res.success = true;
        return true;
    }

    std::vector<sensor_msgs::JointState> currentStates(4);

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
    std::vector<double> zeroArmVector(7);
    zeroArmVector[0] = 0.0;
    zeroArmVector[1] = 0.0;
    zeroArmVector[2] = 0.0;
    zeroArmVector[3] = 0.0;
    zeroArmVector[4] = 0.0;
    zeroArmVector[5] = 0.0;
    zeroArmVector[6] = 0.0;

    std::vector<double> zeroTorsoVector(1);
    zeroTorsoVector[0] = 0.0;

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
                if (msgTraj.points[i].velocities.size() > j + leftJointsStart) {
                    leftVelocities[j] = msgTraj.points[i].velocities[j + leftJointsStart];
                } else {
                    leftVelocities[j] = 0;
                }
            }
            leftGroupPoints[i].group_number = 0;
            leftGroupPoints[i].num_joints = 7;
            leftGroupPoints[i].positions = leftPositions;
            leftGroupPoints[i].velocities = leftVelocities;
            leftGroupPoints[i].time_from_start = msgTraj.points[i].time_from_start;
        }
    }else {
        leftSize = msgSize;
        leftGroupPoints.resize(msgSize);
        for(i = 0; i < msgSize; i++) {
            leftGroupPoints[i].group_number = 0;
            leftGroupPoints[i].num_joints = 7;
            leftGroupPoints[i].positions = jointStates[0].position;
            leftGroupPoints[i].velocities = zeroArmVector;
            leftGroupPoints[i].time_from_start = msgTraj.points[i].time_from_start;
        }
    }

    if(moveRight) {
        rightSize = msgSize;
        rightGroupPoints.resize(rightSize);
        for(i = 0; i < rightSize; i++) {
            for(j = 0; j < 7; j++) {
                rightPositions[j] = msgTraj.points[i].positions[j + rightJointsStart];
            }
            for(j = 0; j < 7; j++) {
                if (msgTraj.points[i].velocities.size() > j + rightJointsStart) {
                    rightVelocities[j] = msgTraj.points[i].velocities[j + rightJointsStart];
                } else {
                    rightVelocities[j] = 0;
                }
            }
            rightGroupPoints[i].group_number = 1;
            rightGroupPoints[i].num_joints = 7;
            rightGroupPoints[i].positions = rightPositions;
            rightGroupPoints[i].velocities = rightVelocities;
            rightGroupPoints[i].time_from_start = msgTraj.points[i].time_from_start;
        }
    }else {
        rightSize = msgSize;

        rightGroupPoints.resize(msgSize);
        for(i = 0; i < msgSize; i++) {
            rightGroupPoints[i].group_number = 1;
            rightGroupPoints[i].num_joints = 7;
            rightGroupPoints[i].positions = jointStates[1].position;
            rightGroupPoints[i].velocities = zeroArmVector;
            rightGroupPoints[i].time_from_start = msgTraj.points[i].time_from_start;
        }
    }

    if(moveTorso) {
        torsoSize = msgSize;
        torso1GroupPoints.resize(torsoSize);
        torso2GroupPoints.resize(torsoSize);
        for(i = 0; i < torsoSize; i++) {
            torsoPositions[0] = msgTraj.points[i].positions[torsoJointsStart];
            if (msgTraj.points[i].velocities.size() > torsoJointsStart) {
                torsoVelocities[0] = msgTraj.points[i].velocities[torsoJointsStart];
            } else {
                torsoVelocities[0] = 0;
            }

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
        torsoSize = msgSize;
        torso1GroupPoints.resize(msgSize);
        for(i = 0; i < msgSize; i++) {
            torso1GroupPoints[i].group_number = 2;
            torso1GroupPoints[i].num_joints = 1;
            torso1GroupPoints[i].positions = jointStates[2].position;
            torso1GroupPoints[i].velocities = zeroTorsoVector;
            torso1GroupPoints[i].time_from_start = msgTraj.points[i].time_from_start;
        }

        torso2GroupPoints.resize(msgSize);
        for(i = 0; i < msgSize; i++) {
            torso2GroupPoints[i].group_number = 3;
            torso2GroupPoints[i].num_joints = 1;
            torso2GroupPoints[i].positions = jointStates[3].position;
            torso2GroupPoints[i].velocities = zeroTorsoVector;
            torso2GroupPoints[i].time_from_start = msgTraj.points[i].time_from_start;
        }
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
        trajPoints[i].num_groups = 4;
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
    // std::cout << "##########   TRAJECTORY:   ##########" << std::endl;
    // std::cout << completeTraj << std::endl;

    // Execute trajectory
    motoman_msgs::CmdJointTrajectoryEx cmd;
    cmd.request.trajectory = completeTraj;
    jointPathCommand.call(cmd);

    // // sleep for expected duration of trajectory (rounded up to nearest second)
    double trajDuration =  msgTraj.points[msgSize-1].time_from_start.toSec();
    // for (int i = 0; i < (int)trajDuration; i++) {
    //     sleep(1.0);
    //     std::cout << i + 1 << " seconds" << std::endl;
    // }

    ros::Time begin = ros::Time::now();

    // Wait until current joint state is within tolerance of target state
    std::vector<double> endPositions = req.jointTraj.points[msgSize-1].positions;

    std::vector<std::string> endJointNames = req.jointTraj.joint_names;

    ROS_INFO("##########  endPositions: %f", endPositions[0]);
    ROS_INFO("##########  endPositions: %f", endPositions[1]);
    ROS_INFO("##########  endPositions: %f", endPositions[2]);
    ROS_INFO("##########  endPositions: %f", endPositions[3]);
    ROS_INFO("##########  endPositions: %f", endPositions[4]);
    ROS_INFO("##########  endPositions: %f", endPositions[5]);
    ROS_INFO("##########  endPositions: %f", endPositions[6]);
    ROS_INFO("##########  endPositions: %f", endPositions[7]);
    ROS_INFO("##########  endPositions: %f", endPositions[8]);

    int joint_iter;
    double tolerance = 0.001;  // joint tolerance in rads
    double timeBuffer = 5.0;
    double timeLimit = trajDuration + timeBuffer;
    bool finished_trajectory = false;
    bool timedOut = false;
    double currentPosition;
    std::vector<double> jointPositions(8);
    moveit::planning_interface::MoveGroup move_group("sda10f");
    while( !finished_trajectory ) {
        if( ros::Time::now().toSec() > (begin.toSec() + timeLimit) ) {
            ROS_INFO("Trajectory timed out");
            res.success = false;
            timedOut = true;
            break;
        }

        sleep(0.1);
        // std::vector<double> currentPositions = move_group.getCurrentState()->joint_state.position;
        // std::cout << "##########   currentPositions: " << currentPositions << std::endl;
        jointPositions[0] = jointStates[2].position[0];
        if (moveLeft) {
            jointPositions[1] = jointStates[0].position[0];
            jointPositions[2] = jointStates[0].position[1];
            jointPositions[3] = jointStates[0].position[2];
            jointPositions[4] = jointStates[0].position[3];
            jointPositions[5] = jointStates[0].position[4];
            jointPositions[6] = jointStates[0].position[5];
            jointPositions[7] = jointStates[0].position[6];
        }
        else if (moveRight) {
            jointPositions[1] = jointStates[1].position[0];
            jointPositions[2] = jointStates[1].position[1];
            jointPositions[3] = jointStates[1].position[2];
            jointPositions[4] = jointStates[1].position[3];
            jointPositions[5] = jointStates[1].position[4];
            jointPositions[6] = jointStates[1].position[5];
            jointPositions[7] = jointStates[1].position[6];
        }

        finished_trajectory = true;
        for( joint_iter = 0; joint_iter < endJointNames.size(); joint_iter++ ) {
            currentPosition = jointPositions[joint_iter];
            // ROS_INFO( "currentPosition: %f", currentPosition);
            // ROS_INFO( "difference: %f", currentPosition - endPositions[joint_iter]);
            if( fabs(currentPosition - endPositions[joint_iter]) > tolerance ) {
                // ROS_INFO("finished_trajectory = false");
                finished_trajectory = false;
            }
        }
    }

    for( joint_iter = 0; joint_iter < endJointNames.size(); joint_iter++ ) {
        currentPosition = jointPositions[joint_iter];
        ROS_INFO( "currentPosition: %f", currentPosition);
        ROS_INFO( "difference: %f", currentPosition - endPositions[joint_iter]);
        sleep(0.1);
    }

    std::cout << "Start Time: " << begin.sec << std::endl;
    std::cout << "End Time: " << ros::Time::now().sec <<  std::endl;

    std::cout << cmd.response << std::endl;
    std::cout << "##########   COMPLETED TRAJECTORY   ##########" << std::endl;

    if (timedOut) {
        res.success = false;
        return true;
    }

    res.success = cmd.response.code.val == 1;
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

    jointPathCommand = node_handle.serviceClient<motoman_msgs::CmdJointTrajectoryEx>("/joint_path_command");

    ros::Subscriber jointStateSub = node_handle.subscribe("/joint_states", 0, jointStateCallback);

    ros::ServiceServer service = node_handle.advertiseService("convert_trajectory_service", move_callback);
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}
