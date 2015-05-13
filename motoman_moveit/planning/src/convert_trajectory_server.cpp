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
#include <mutex>
#include <motoman_moveit/convert_trajectory_server.h>

#define TARGET_REACHED_THRESHOLD 0.0001
#define EXEC_TIME_BUFFER 2.0

// Globals
ros::ServiceClient g_execute_client;
std::vector<sensor_msgs::JointState> g_joint_states(4);
std::mutex g_joint_state_mutex;

std::map<std::string, double> GenerateNamePositionMap(const std::vector<std::string>& joint_names, const std::vector<double>& joint_positions)
{
    if (joint_names.size() != joint_positions.size())
    {
        throw std::invalid_argument("Size of joint names and joint points do not match");
    }
    std::map<std::string, double> name_position_map;
    for (size_t idx = 0; idx < joint_names.size(); idx++)
    {
        name_position_map[joint_names[idx]] = joint_positions[idx];
    }
    return name_position_map;
}

inline double GetJointPosition(const std::map<std::string, double>& joint_state_map, const std::map<std::string, double>& command_map, const std::string joint_name)
{
    // Check if the joint is in the command map - if so, we use the command value
    std::map<std::string, double>::const_iterator found_in_command_itr = command_map.find(joint_name);
    if (found_in_command_itr != command_map.end())
    {
        return found_in_command_itr->second;
    }
    // If not, we check the joint state map
    std::map<std::string, double>::const_iterator found_in_state_itr = joint_state_map.find(joint_name);
    if (found_in_state_itr != joint_state_map.end())
    {
        return found_in_state_itr->second;
    }
    // If we haven't found it
    throw std::invalid_argument("Cound not find joint in either joint state or command");
}

std::vector<double> GetGroupPositions(const std::map<std::string, double>& joint_state_map, const std::map<std::string, double>& command_map, const std::vector<std::string>& group_joint_names)
{
    std::vector<double> group_positions(group_joint_names.size());
    for (size_t idx = 0; idx < group_joint_names.size(); idx++)
    {
        const std::string joint_name = group_joint_names[idx];
        group_positions[idx] = GetJointPosition(joint_state_map, command_map, joint_name);
    }
    return group_positions;
}

std::vector<motoman_msgs::DynamicJointsGroup> BuildGroupPoints(const std::map<std::string, double>& joint_state_map, const trajectory_msgs::JointTrajectory& commanded_traj, const std::vector<std::string>& group_joint_names, const int16_t group_number)
{
    std::vector<motoman_msgs::DynamicJointsGroup> group_points;
    for (size_t idx = 0; idx < commanded_traj.points.size(); idx++)
    {
        const trajectory_msgs::JointTrajectoryPoint& commanded_point = commanded_traj.points[idx];
        std::map<std::string, double> commanded_joint_map = GenerateNamePositionMap(commanded_traj.joint_names, commanded_point.positions);
        motoman_msgs::DynamicJointsGroup group_point;
        group_point.group_number = group_number;
        group_point.time_from_start = commanded_point.time_from_start;
        group_point.num_joints = (int16_t)group_joint_names.size();
        // Populate the point
        group_point.positions = GetGroupPositions(joint_state_map, commanded_joint_map, group_joint_names);
        group_point.velocities = std::vector<double>(group_joint_names.size(), 0.0);
        //group_point.accelerations = ;
        //group_point.effort = ;
    }
    return group_points;
}

bool move_callback(motoman_moveit::convert_trajectory_server::Request &req, motoman_moveit::convert_trajectory_server::Response &res)
{
    if (req.jointTraj.points.size() == 0)
    {
        ROS_WARN("Received empty trajectory, not doing anything.");
        res.success = true;
        return true;
    }
    // Copy the current joint states
    g_joint_state_mutex.lock();
    std::vector<sensor_msgs::JointState> current_states = g_joint_states;
    g_joint_state_mutex.unlock();
    // Insert all joints states into a map
    std::map<std::string, double> current_joint_values;
    for (size_t sdx = 0; sdx < current_states.size(); sdx++)
    {
        const sensor_msgs::JointState& current_state = current_states[sdx];
        if (current_state.name.size() != current_state.position.size())
        {
            ROS_ERROR("Invalid joint state for group %zu", sdx);
            res.success = false;
            return true;
        }
        for (size_t idx = 0; idx < current_state.name.size(); idx++)
        {
            std::string joint_name = current_state.name[idx];
            double joint_position = current_state.position[idx];
            current_joint_values[joint_name] = joint_position;
        }
    }
    const std::vector<std::string>& trajectory_joint_names = req.jointTraj.joint_names;
    // First, safety check the trajectory
    for (size_t idx = 0; idx < req.jointTraj.points.size(); idx++)
    {
        const trajectory_msgs::JointTrajectoryPoint& traj_point = req.jointTraj.points[idx];
        if (traj_point.positions.size() != trajectory_joint_names.size())
        {
            ROS_ERROR("Joint names and joint positions do not match - %zu joint names, %zu joint positions", trajectory_joint_names.size(), traj_point.positions.size());
            res.success = false;
            return true;
        }
    }
    // Now, build a trajectory for each group
    std::vector<std::string> left_arm_group_names(7);
    left_arm_group_names[0] = "arm_left_joint_1_s";
    left_arm_group_names[1] = "arm_left_joint_2_l";
    left_arm_group_names[2] = "arm_left_joint_3_e";
    left_arm_group_names[3] = "arm_left_joint_4_u";
    left_arm_group_names[4] = "arm_left_joint_5_r";
    left_arm_group_names[5] = "arm_left_joint_6_b";
    left_arm_group_names[6] = "arm_left_joint_7_t";
    std::vector<motoman_msgs::DynamicJointsGroup> left_arm_group_points = BuildGroupPoints(current_joint_values, req.jointTraj, left_arm_group_names, 0);
    std::vector<std::string> right_arm_group_names(7);
    right_arm_group_names[0] = "arm_right_joint_1_s";
    right_arm_group_names[1] = "arm_right_joint_2_l";
    right_arm_group_names[2] = "arm_right_joint_3_e";
    right_arm_group_names[3] = "arm_right_joint_4_u";
    right_arm_group_names[4] = "arm_right_joint_5_r";
    right_arm_group_names[5] = "arm_right_joint_6_b";
    right_arm_group_names[6] = "arm_right_joint_7_t";
    std::vector<motoman_msgs::DynamicJointsGroup> right_arm_group_points = BuildGroupPoints(current_joint_values, req.jointTraj, right_arm_group_names, 1);
    std::vector<std::string> torso_1_group_names(1);
    torso_1_group_names[0] = "torso_joint_b1";
    std::vector<motoman_msgs::DynamicJointsGroup> torso_1_group_points = BuildGroupPoints(current_joint_values, req.jointTraj, torso_1_group_names, 2);
    std::vector<std::string> torso_2_group_names(1);
    torso_2_group_names[0] = "torso_joint_b2";
    std::vector<motoman_msgs::DynamicJointsGroup> torso_2_group_points = BuildGroupPoints(current_joint_values, req.jointTraj, torso_2_group_names, 3);
    // Safety check
    if (left_arm_group_points.size() != right_arm_group_points.size() != torso_1_group_points.size() != torso_2_group_points.size())
    {
        ROS_ERROR("Failure generating trajectories for each group");
        res.success = false;
        return true;
    }
    // Now, combine the points into a single trajectory
    std::vector<motoman_msgs::DynamicJointPoint> complete_trajectory_points;
    for (size_t idx = 0; idx < left_arm_group_points.size(); idx++)
    {
        motoman_msgs::DynamicJointPoint new_traj_point;
        new_traj_point.groups = {left_arm_group_points[idx], right_arm_group_points[idx], torso_1_group_points[idx], torso_2_group_points[idx]};
        new_traj_point.num_groups = 4;
    }
    // Assemble full trajectory
    std::vector<std::string> complete_joint_names(16);
    complete_joint_names[0] = "arm_left_joint_1_s";
    complete_joint_names[1] = "arm_left_joint_2_l";
    complete_joint_names[2] = "arm_left_joint_3_e";
    complete_joint_names[3] = "arm_left_joint_4_u";
    complete_joint_names[4] = "arm_left_joint_5_r";
    complete_joint_names[5] = "arm_left_joint_6_b";
    complete_joint_names[6] = "arm_left_joint_7_t";
    complete_joint_names[7] = "arm_right_joint_1_s";
    complete_joint_names[8] = "arm_right_joint_2_l";
    complete_joint_names[9] = "arm_right_joint_3_e";
    complete_joint_names[10] = "arm_right_joint_4_u";
    complete_joint_names[11] = "arm_right_joint_5_r";
    complete_joint_names[12] = "arm_right_joint_6_b";
    complete_joint_names[13] = "arm_right_joint_7_t";
    complete_joint_names[14] = "torso_joint_b1";
    complete_joint_names[15] = "torso_joint_b2";
    motoman_msgs::DynamicJointTrajectory complete_trajectory;
    complete_trajectory.header.frame_id = "/base_link";
    complete_trajectory.joint_names = complete_joint_names;
    complete_trajectory.points = complete_trajectory_points;
    // Execute trajectory
    motoman_msgs::CmdJointTrajectoryEx::Request trajectory_req;
    trajectory_req.trajectory = complete_trajectory;
    motoman_msgs::CmdJointTrajectoryEx::Response trajectory_res;
    bool status = g_execute_client.call(trajectory_req, trajectory_res);
    if (!status)
    {
        ROS_ERROR("Trajectory execution service call failed");
        res.success = false;
        return true;
    }
    if (trajectory_res.code.val != industrial_msgs::ServiceReturnCode::SUCCESS)
    {
        ROS_ERROR("Trajectory execution service call returned FAILURE");
        res.success = false;
        return true;
    }
    else
    {
        ROS_INFO("Trajectory execution service call returned SUCCESS, waiting for execution to finish");
    }
    // Wait for the trajectory to finish, plus a little buffer time
    ros::Duration trajectory_exec_duration = req.jointTraj.points[req.jointTraj.points.size() - 1].time_from_start + ros::Duration(EXEC_TIME_BUFFER);
    trajectory_exec_duration.sleep();
    // Now, check to see if we've reached the target
    // Copy the current joint states
    g_joint_state_mutex.lock();
    std::vector<sensor_msgs::JointState> check_current_states = g_joint_states;
    g_joint_state_mutex.unlock();
    // Insert all joints states into a map
    std::map<std::string, double> check_current_joint_values;
    for (size_t sdx = 0; sdx < current_states.size(); sdx++)
    {
        const sensor_msgs::JointState& check_current_state = check_current_states[sdx];
        if (check_current_state.name.size() != check_current_state.position.size())
        {
            ROS_ERROR("Invalid joint state for group %zu when trying to check if trajectory execution finished", sdx);
            res.success = false;
            return true;
        }
        for (size_t idx = 0; idx < check_current_state.name.size(); idx++)
        {
            std::string joint_name = check_current_state.name[idx];
            double joint_position = check_current_state.position[idx];
            check_current_joint_values[joint_name] = joint_position;
        }
    }
    // Get an equivalent map for the target joint states
    std::map<std::string, double> target_joint_values = GenerateNamePositionMap(req.jointTraj.joint_names, req.jointTraj.points[req.jointTraj.points.size() - 1].positions);
    // Make sure the target is reached within the desired tolerance
    std::map<std::string, double>::const_iterator target_joint_values_itr;
    for (target_joint_values_itr = target_joint_values.begin(); target_joint_values_itr != target_joint_values.end(); ++target_joint_values_itr)
    {
        std::string joint_name = target_joint_values_itr->first;
        double target_joint_value = target_joint_values_itr->second;
        // Look up the joint in the joint state
        std::map<std::string, double>::const_iterator found_state_value = check_current_joint_values.find(joint_name);
        if (found_state_value != check_current_joint_values.end())
        {
            double state_joint_value = found_state_value->second;
            double joint_value_delta = fabs(target_joint_value - state_joint_value);
            if (joint_value_delta >= TARGET_REACHED_THRESHOLD)
            {
                ROS_ERROR("Joint %s did not reach target value %f, is at %f instead", joint_name.c_str(), target_joint_value, state_joint_value);
                res.success = false;
                return true;
            }
        }
        else
        {
            ROS_ERROR("Couldn't find joint state value for joint %s when trying to check if trajectory execution finished", joint_name.c_str());
            res.success = false;
            return true;
        }
    }
    ROS_INFO("Target reached");
    res.success = true;
    return true;
}

// Callback to get joint values from current robot state
void jointStateCallback(sensor_msgs::JointState msg)
{
    g_joint_state_mutex.lock();
    if(msg.name[0] == "arm_left_joint_1_s")
    {
        g_joint_states[0] = msg;
    }
    if(msg.name[0] == "arm_right_joint_1_s")
    {
        g_joint_states[1] = msg;
    }
    if(msg.name[0] == "torso_joint_b1")
    {
        g_joint_states[2] = msg;
    }
    if(msg.name[0] == "torso_joint_b2")
    {
        g_joint_states[3] = msg;
    }
    g_joint_state_mutex.unlock();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "convert_trajectory_server");
    ros::NodeHandle node_handle;
    g_execute_client = node_handle.serviceClient<motoman_msgs::CmdJointTrajectoryEx>("/joint_path_command");
    ros::Subscriber jointStateSub = node_handle.subscribe("joint_states", 0, jointStateCallback);
    ros::ServiceServer service = node_handle.advertiseService("convert_trajectory_service", move_callback);
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    return 0;
}
