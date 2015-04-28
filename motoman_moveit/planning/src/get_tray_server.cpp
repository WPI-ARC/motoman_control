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
#include <gripper_srv/gripper.h>


bool move_callback(motoman_moveit::get_tray_server::Request &req,
                   motoman_moveit::get_tray_server::Response &res) {

    if(!(req.hand == "left" || req.hand == "right")) {
        std::cout << "##########   INVALID REQUEST   ##########" << std::endl;
        std::cout << "##########   SERVICE FAILED   ##########" << std::endl;
        res.success = false;
        return 0;
    }

    bool isLeft = (req.hand == "left");

    std::cout << "PLANNING..." << std::endl;

//    moveit::planning_interface::MoveGroup move_group("arm_" + req.hand + "_torso");
    moveit::planning_interface::MoveGroup move_group("arm_" + req.hand);

    move_group.setPlannerId("RRTstarkConfigDefault");
    move_group.setPlanningTime(5.0);
    move_group.setStartStateToCurrentState();
    geometry_msgs::Pose targetPose;
    gripper_srv::gripper gripperServer;

    if(isLeft){
        // OPEN LEFT HAND
        gripperServer.request.command = "command: '70'";
        ros::service::call("command_gripper", gripperServer);

        // GO TO INITIAL POSE
//        targetPose.position.x = 0.735592;
//        targetPose.position.z = 0.446805;
//        targetPose.position.y = 1.18608;
//        targetPose.orientation.x = -0.108456;
//        targetPose.orientation.y = 0.708157;
//        targetPose.orientation.z = 0.693177;
//        targetPose.orientation.w = 0.0790984;

        targetPose.position.x = 0.429567;
        targetPose.position.y = 0.859446;
        targetPose.position.z = 1.25832;
        targetPose.orientation.x = -0.673667;
        targetPose.orientation.y = 0.207501;
        targetPose.orientation.z = 0.706735;
        targetPose.orientation.w = -0.0603374;


    }else {
        // OPEN RIGHT HAND
        gripperServer.request.command = "command: '70'";
        ros::service::call("command_gripper", gripperServer);

        // GO TO INITIAL POSE
        /*
         * fill in pose info
         */
        targetPose.position.x = 0.0;
        targetPose.position.z = 0.0;
        targetPose.position.y = 0.0;
        targetPose.orientation.x = 0.0;
        targetPose.orientation.y = 0.0;
        targetPose.orientation.z = 0.0;
        targetPose.orientation.w = 1.0;

    }

    move_group.setPoseTarget(targetPose);

    moveit::planning_interface::MoveGroup::Plan plan;
    move_group.plan(plan);

    std::cout << "##########   PLANNED PLAN   ##########" << std::endl;
    //std::cout << plan.trajectory_.joint_trajectory << std::endl;

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

    std::vector<double> zeroVector(7);
    zeroVector[0] = 0.0;
    zeroVector[1] = 0.0;
    zeroVector[2] = 0.0;
    zeroVector[3] = 0.0;
    zeroVector[4] = 0.0;
    zeroVector[5] = 0.0;
    zeroVector[6] = 0.0;


    std::vector<trajectory_msgs::JointTrajectoryPoint> trajPoints;

    int planSize = plan.trajectory_.joint_trajectory.points.size();
    trajPoints.resize(planSize);

    for(int i = 0; i< planSize; i++) {
        trajPoints[i].positions = plan.trajectory_.joint_trajectory.points[i].positions;
        trajPoints[i].velocities = plan.trajectory_.joint_trajectory.points[i].velocities;
        trajPoints[i].time_from_start = plan.trajectory_.joint_trajectory.points[i].time_from_start;
    }

    trajectory_msgs::JointTrajectory jointTraj;
    jointTraj.header.frame_id = "/base_link";

    if(isLeft)
        jointTraj.joint_names = leftJointNames;
    else
        jointTraj.joint_names = rightJointNames;

    jointTraj.points = trajPoints;

    motoman_moveit::convert_trajectory_server move0;
    move0.request.jointTraj = jointTraj;
    std::cout << "REQUEST: " << move0.request << std::endl;
    ros::service::call("convert_trajectory_service", move0);

    std::cout << "##########   SLEEPING   ##########" << std::endl;

    sleep(trajPoints[trajPoints.size()-1].time_from_start.toSec() + 0.1); //0.1 add for buffer

    // FOLLOW CARTESIAN PATH TO PLACE ROBOTIQ FINGERS AROUND TRAY
    std::vector<geometry_msgs::Pose> cartesianPoses1(2);
    cartesianPoses1[0] = targetPose;
    cartesianPoses1[1] = targetPose;
    if(isLeft)
        cartesianPoses1[1].position.y -= 0.16/*SOME NUMBER*/;
    else
        cartesianPoses1[1].position.y += 0.16/*SOME NUMBER*/;

    moveit_msgs::RobotTrajectory cartesianRobotTraj1;
    move_group.computeCartesianPath(cartesianPoses1, 0.01, 0.0, cartesianRobotTraj1);

    std::cout << "##########   COMPUTED CARTESIAN PATH 1   ##########" << std::endl;

    trajectory_msgs::JointTrajectory cartesianTraj1;
    cartesianTraj1 = cartesianRobotTraj1.joint_trajectory;

    motoman_moveit::convert_trajectory_server move1;
    move1.request.jointTraj = cartesianTraj1;
    ros::service::call("convert_trajectory_service", move1);

    sleep(cartesianTraj1.points[1].time_from_start.toSec() + 0.1); //0.1 add for buffer

    std::cout << "##########   MOVED TO POSE #2   ##########" << std::endl;

    if(isLeft) {
        // CLOSE LEFT HAND
        gripperServer.request.command = "command: 'close'";
        ros::service::call("command_gripper", gripperServer);
    }else {
        // CLOSE RIGHT HAND
        gripperServer.request.command = "command: 'close'";
        ros::service::call("command_gripper", gripperServer);
    }

    // FOLLOW CARTESIAN PATH TO MOVE TRAY AWAY FROM OBSTACLES
    std::vector<geometry_msgs::Pose> cartesianPoses2(2);
    cartesianPoses2[0] = cartesianPoses1[1];
    cartesianPoses2[1] = cartesianPoses1[1];
    cartesianPoses2[1].position.x -= 0.5/*SOME NUMBER*/;
    cartesianPoses2[1].position.z += 0.1/*SOME NUMBER*/;

    moveit_msgs::RobotTrajectory cartesianRobotTraj2;
    move_group.computeCartesianPath(cartesianPoses2, 0.01, 0.0, cartesianRobotTraj2);

    std::cout << "##########   COMPUTED CARTESIAN PATH 2   ##########" << std::endl;

    trajectory_msgs::JointTrajectory cartesianTraj2;
    cartesianTraj2 = cartesianRobotTraj2.joint_trajectory;

    motoman_moveit::convert_trajectory_server move2;
    move2.request.jointTraj = cartesianTraj2;
    ros::service::call("convert_trajectory_service", move2);

    sleep(cartesianTraj2.points[1].time_from_start.toSec() + 0.1); //0.1 add for buffer

    std::cout << "##########   MOVED TO POSE #2   ##########" << std::endl;
    std::cout << "##########   DONE   ##########" << std::endl;
    res.success = true;
    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "get_tray_server");
    ros::NodeHandle node_handle;

    ros::ServiceServer service = node_handle.advertiseService("get_tray_service", move_callback);
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}
