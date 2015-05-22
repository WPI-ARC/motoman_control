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

#define TARGET_REACHED_THRESHOLD 0.01
#define TRAJECTORY_START_THRESHOLD 0.01
#define EXEC_TIME_BUFFER 1.0
#define EXEC_TIME_FRACTION 1.25
#define REMOVE_TRAJECTORY_VELOCITIES true

// Globals
bool g_simulation_execution = false;
ros::Publisher g_simulation_execution_pub;
ros::ServiceClient g_execute_client;
std::vector<sensor_msgs::JointState> g_joint_states(4);
std::mutex g_joint_state_mutex;

std::map<std::string, double> GenerateNameValueMap(const std::vector<std::string>& joint_names, const std::vector<double>& joint_positions, bool allow_default=false)
{
    if (joint_names.size() != joint_positions.size())
    {
        if (allow_default)
        {
            ROS_WARN("Names and values do not match, replacing values with default");
            std::map<std::string, double> name_position_map;
            for (size_t idx = 0; idx < joint_names.size(); idx++)
            {
                name_position_map[joint_names[idx]] = 0.0;
            }
            return name_position_map;
        }
        else
        {
            throw std::invalid_argument("Size of joint names and joint points do not match");
        }
    }
    std::map<std::string, double> name_position_map;
    for (size_t idx = 0; idx < joint_names.size(); idx++)
    {
        name_position_map[joint_names[idx]] = joint_positions[idx];
    }
    return name_position_map;
}

inline double GetJointPosition(const std::map<std::string, double>& joint_state_map, const std::map<std::string, double>& command_map, const std::string joint_name, const bool use_state_only)
{
    if (use_state_only)
    {
        std::map<std::string, double>::const_iterator found_in_state_itr = joint_state_map.find(joint_name);
        if (found_in_state_itr != joint_state_map.end())
        {
            return found_in_state_itr->second;
        }
        // If we haven't found it
        throw std::invalid_argument("Cound not find joint in joint state [use_state_only = true]");
    }
    else
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
}

std::vector<double> GetGroupPositions(const std::map<std::string, double>& joint_state_map, const std::map<std::string, double>& command_map, const std::vector<std::string>& group_joint_names, const bool use_state_only=false)
{
    std::vector<double> group_positions(group_joint_names.size());
    for (size_t idx = 0; idx < group_joint_names.size(); idx++)
    {
        const std::string joint_name = group_joint_names[idx];
        group_positions[idx] = GetJointPosition(joint_state_map, command_map, joint_name, use_state_only);
    }
    return group_positions;
}

std::vector<double> GetGroupVelocities(const std::map<std::string, double>& command_map, const std::vector<std::string>& group_joint_names)
{
    if (REMOVE_TRAJECTORY_VELOCITIES)
    {
        return std::vector<double>(group_joint_names.size(), 0.0);
    }
    else
    {
        std::vector<double> group_velocities(group_joint_names.size());
        for (size_t idx = 0; idx < group_joint_names.size(); idx++)
        {
            const std::string joint_name = group_joint_names[idx];
            std::map<std::string, double>::const_iterator found_joint_velocity = command_map.find(joint_name);
            if (found_joint_velocity != command_map.end())
            {
                group_velocities[idx] = found_joint_velocity->second;
            }
            else
            {
                group_velocities[idx] = 0.0;
            }
        }
        return group_velocities;
    }
}

std::vector<motoman_msgs::DynamicJointsGroup> BuildGroupPoints(const std::map<std::string, double>& joint_state_map, const trajectory_msgs::JointTrajectory& commanded_traj, const std::vector<std::string>& group_joint_names, const int16_t group_number)
{
    std::vector<motoman_msgs::DynamicJointsGroup> group_points;
    for (size_t idx = 0; idx < commanded_traj.points.size(); idx++)
    {
        const trajectory_msgs::JointTrajectoryPoint& commanded_point = commanded_traj.points[idx];
        std::map<std::string, double> commanded_position_map = GenerateNameValueMap(commanded_traj.joint_names, commanded_point.positions);
        std::map<std::string, double> commanded_velocity_map = GenerateNameValueMap(commanded_traj.joint_names, commanded_point.velocities, true);
        motoman_msgs::DynamicJointsGroup group_point;
        group_point.group_number = group_number;
        group_point.time_from_start = (commanded_point.time_from_start * EXEC_TIME_FRACTION);
        group_point.num_joints = (int16_t)group_joint_names.size();
        if (idx == 0)
        {
            // Populate the point
            group_point.positions = GetGroupPositions(joint_state_map, commanded_position_map, group_joint_names, true);
            group_point.velocities = GetGroupVelocities(commanded_velocity_map, group_joint_names);
        }
        else
        {
            // Populate the point
            group_point.positions = GetGroupPositions(joint_state_map, commanded_position_map, group_joint_names);
            group_point.velocities = GetGroupVelocities(commanded_velocity_map, group_joint_names);
        }
        group_points.push_back(group_point);
    }
    return group_points;
}

std::pair<double, int64_t> GetMaxVelocityJoint(const trajectory_msgs::JointTrajectoryPoint& current_point)
{
    double max_velocity = 0.0;
    int64_t max_index = -1;
    for (size_t jdx = 0; jdx < current_point.velocities.size(); jdx++)
    {
        double joint_velocity = fabs(current_point.velocities[jdx]);
        if (joint_velocity > max_velocity)
        {
            max_velocity = joint_velocity;
            max_index = (int64_t)jdx;
        }
    }
    return std::pair<double, int64_t>(max_velocity, max_index);
}

std::pair<double, int64_t> GetMaxFiniteDifferenceVelocityJoint(const trajectory_msgs::JointTrajectoryPoint& previous_point, const trajectory_msgs::JointTrajectoryPoint& current_point)
{
    if (previous_point.positions.size() != current_point.positions.size())
    {
        throw std::invalid_argument("JointTrajectoryPoint sizes do not match");
    }
    double max_velocity = 0.0;
    int64_t max_index = -1;
    for (size_t jdx = 0; jdx < current_point.positions.size(); jdx++)
    {
        double previous_position = previous_point.positions[jdx];
        double current_position = current_point.positions[jdx];
        double joint_velocity = fabs(current_position - previous_position);
        if (joint_velocity > max_velocity)
        {
            max_velocity = joint_velocity;
            max_index = (int64_t)jdx;
        }
    }
    return std::pair<double, int64_t>(max_velocity, max_index);
}

bool CheckVelocities(const trajectory_msgs::JointTrajectory& traj)
{
    // We can't do finite differencing with one point, and besides, this will be set to the current config anyways
    if (traj.points.size() <= 1)
    {
        ROS_WARN("Single point trajectory, cannot meaningfully check velocities");
        return true;
    }
    else
    {
        double max_velocity = 0.0;
        std::string max_velocity_joint = "unknown";
        for (size_t idx = 1; idx < traj.points.size(); idx++)
        {
            const trajectory_msgs::JointTrajectoryPoint& previous_point = traj.points[idx - 1];
            const trajectory_msgs::JointTrajectoryPoint& current_point = traj.points[idx];
            std::pair<double, int64_t> finite_difference_check = GetMaxFiniteDifferenceVelocityJoint(previous_point, current_point);
            if (traj.joint_names.size() == current_point.velocities.size())
            {
                std::pair<double, int64_t> max_velocity_check = GetMaxVelocityJoint(current_point);
                if (max_velocity_check.first > finite_difference_check.first && max_velocity_check.first > max_velocity)
                {
                    max_velocity = max_velocity_check.first;
                    max_velocity_joint = traj.joint_names[(size_t)max_velocity_check.second];
                }
                else if (finite_difference_check.first > max_velocity_check.first && finite_difference_check.first > max_velocity)
                {
                    max_velocity = finite_difference_check.first;
                    max_velocity_joint = traj.joint_names[(size_t)finite_difference_check.second];
                }
            }
            else
            {
                if (finite_difference_check.first > max_velocity)
                {
                    max_velocity = finite_difference_check.first;
                    max_velocity_joint = traj.joint_names[(size_t)finite_difference_check.second];
                }
            }
        }
        ROS_INFO("Maximum velocity joint is [%s] with velocity %f", max_velocity_joint.c_str(), max_velocity);
        return true;
    }
}

bool move_callback(motoman_moveit::convert_trajectory_server::Request &req, motoman_moveit::convert_trajectory_server::Response &res)
{
    if (req.jointTraj.points.size() == 0)
    {
        ROS_WARN("Received empty trajectory, not doing anything.");
        res.success = true;
        return true;
    }
    ROS_INFO("Received trajectory with %zu points to convert and execute", req.jointTraj.points.size());
    if (REMOVE_TRAJECTORY_VELOCITIES)
    {
        ROS_WARN("Trajectory velocities will be removed before execution");
    }
    ROS_INFO("Trajectory points will be extended by %f%%", ((EXEC_TIME_FRACTION - 1.0) * 100.0));
    // Copy the current joint states
    g_joint_state_mutex.lock();
    std::vector<sensor_msgs::JointState> current_states = g_joint_states;
    g_joint_state_mutex.unlock();
    ROS_INFO("Got current joint state");
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
    // Check to make sure the start is within tolerance
    std::map<std::string, double> start_joint_values = GenerateNameValueMap(req.jointTraj.joint_names, req.jointTraj.points[0].positions);
    // Make sure the target is reached within the desired tolerance
    std::map<std::string, double>::const_iterator start_joint_values_itr;
    for (start_joint_values_itr = start_joint_values.begin(); start_joint_values_itr != start_joint_values.end(); ++start_joint_values_itr)
    {
        std::string joint_name = start_joint_values_itr->first;
        double start_joint_value = start_joint_values_itr->second;
        // Look up the joint in the joint state
        std::map<std::string, double>::const_iterator found_state_value = current_joint_values.find(joint_name);
        if (found_state_value != current_joint_values.end())
        {
            double state_joint_value = found_state_value->second;
            double joint_value_delta = fabs(start_joint_value - state_joint_value);
            if (joint_value_delta >= TRAJECTORY_START_THRESHOLD)
            {
                ROS_ERROR("Joint %s is not close enough to the current configuration %f, is at %f instead", joint_name.c_str(), state_joint_value, start_joint_value);
                res.success = false;
                return true;
            }
        }
        else
        {
            ROS_ERROR("Couldn't find joint state value for joint %s when trying to safety check the trajectory start", joint_name.c_str());
            res.success = false;
            return true;
        }
    }
    // Check the velocities of the trajectory
    bool velocity_check_passed = CheckVelocities(req.jointTraj);
    if (velocity_check_passed)
    {
        ROS_INFO("Trajectory safety check passed");
    }
    else
    {
        ROS_ERROR("Joint velocity safety check failed - Not safe to execute");
        res.success = false;
        return true;
    }
    // Simulation-only execution
    if (g_simulation_execution)
    {
        ROS_INFO("Executing trajectory (simulation)...");
        g_simulation_execution_pub.publish(req.jointTraj);
        ROS_INFO("Trajectory execution started, waiting for execution to finish");
    }
    // Real execution
    else
    {
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
        ROS_INFO("Generated left arm group trajectory with %zu points", left_arm_group_points.size());
        std::vector<std::string> right_arm_group_names(7);
        right_arm_group_names[0] = "arm_right_joint_1_s";
        right_arm_group_names[1] = "arm_right_joint_2_l";
        right_arm_group_names[2] = "arm_right_joint_3_e";
        right_arm_group_names[3] = "arm_right_joint_4_u";
        right_arm_group_names[4] = "arm_right_joint_5_r";
        right_arm_group_names[5] = "arm_right_joint_6_b";
        right_arm_group_names[6] = "arm_right_joint_7_t";
        std::vector<motoman_msgs::DynamicJointsGroup> right_arm_group_points = BuildGroupPoints(current_joint_values, req.jointTraj, right_arm_group_names, 1);
        ROS_INFO("Generated right arm group trajectory with %zu points", right_arm_group_points.size());
        std::vector<std::string> torso_1_group_names(1);
        torso_1_group_names[0] = "torso_joint_b1";
        std::vector<motoman_msgs::DynamicJointsGroup> torso_1_group_points = BuildGroupPoints(current_joint_values, req.jointTraj, torso_1_group_names, 2);
        ROS_INFO("Generated torso 1 group trajectory with %zu points", torso_1_group_points.size());
        std::vector<std::string> torso_2_group_names(1);
        torso_2_group_names[0] = "torso_joint_b2";
        std::vector<motoman_msgs::DynamicJointsGroup> torso_2_group_points = BuildGroupPoints(current_joint_values, req.jointTraj, torso_2_group_names, 3);
        ROS_INFO("Generated torso 2 group trajectory with %zu points", torso_2_group_points.size());
        // Safety check
        size_t expected_size = req.jointTraj.points.size();
        if (left_arm_group_points.size() != expected_size || right_arm_group_points.size() != expected_size || torso_1_group_points.size() != expected_size || torso_2_group_points.size() != expected_size)
        {
            ROS_ERROR("Failure generating trajectories for each group");
            res.success = false;
            return true;
        }
        // Now, combine the points into a single trajectory
        std::vector<motoman_msgs::DynamicJointPoint> complete_trajectory_points;
        for (size_t idx = 0; idx < expected_size; idx++)
        {
            motoman_msgs::DynamicJointPoint new_traj_point;
            new_traj_point.groups = {left_arm_group_points[idx], right_arm_group_points[idx], torso_1_group_points[idx], torso_2_group_points[idx]};
            new_traj_point.num_groups = 4;
            complete_trajectory_points.push_back(new_traj_point);
        }
        ROS_INFO("Assembled combined trajectory with %zu points", complete_trajectory_points.size());
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
        ROS_INFO("Executing trajectory (real)...");
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
    }
    // Wait for the trajectory to finish, plus a little buffer time
    ros::Duration trajectory_exec_duration = (req.jointTraj.points[req.jointTraj.points.size() - 1].time_from_start * EXEC_TIME_FRACTION) + ros::Duration(EXEC_TIME_BUFFER);
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
    std::map<std::string, double> target_joint_values = GenerateNameValueMap(req.jointTraj.joint_names, req.jointTraj.points[req.jointTraj.points.size() - 1].positions);
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
    ros::NodeHandle node_handle_private("~");
    std::string simulation_execution_topic;
    node_handle_private.param(std::string("simulation_execution_topic"), simulation_execution_topic, std::string(""));
    if (simulation_execution_topic != "")
    {
        ROS_INFO("ConvertTrajectoryServer running in simulated execution mode");
        g_simulation_execution_pub = node_handle.advertise<trajectory_msgs::JointTrajectory>(simulation_execution_topic, 1);
        g_simulation_execution = true;
    }
    else
    {
        ROS_INFO("ConvertTrajectoryServer running in real execution mode");
        g_simulation_execution = false;
    }
    g_execute_client = node_handle.serviceClient<motoman_msgs::CmdJointTrajectoryEx>("/joint_path_command");
    ros::Subscriber jointStateSub = node_handle.subscribe("joint_states", 0, jointStateCallback);
    ros::ServiceServer service = node_handle.advertiseService("convert_trajectory_service", move_callback);
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    return 0;
}
