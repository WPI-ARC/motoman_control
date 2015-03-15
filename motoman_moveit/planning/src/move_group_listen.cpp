#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Scalar.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

// currently this function takes a Pose and tells the right end-effector to move there
void right_callback(geometry_msgs::Pose msg) {
    moveit::planning_interface::MoveGroup group_right("arm_right");
    geometry_msgs::Pose target_pose_right;
    geometry_msgs::Pose currentPose;

    currentPose = group_right.getCurrentPose().pose;

    target_pose_right.position.x = msg.position.x;
    target_pose_right.position.y = msg.position.y;
    target_pose_right.position.z = msg.position.z;
    target_pose_right.orientation.x = msg.orientation.x;
    target_pose_right.orientation.y = msg.orientation.y;
    target_pose_right.orientation.z = msg.orientation.z;
    target_pose_right.orientation.w = msg.orientation.w;

    group_right.setPoseTarget(target_pose_right);

    // compute and visualize plan
    moveit::planning_interface::MoveGroup::Plan right_plan;
    bool success = group_right.plan(right_plan);
    
    std::cout << "VISUALIZING..." << "\n";
    
    // Sleep while plan is shown in Rviz
    sleep(5.0);

    std::cout << "MOVING..." << "\n";
    
    // move to target pose
    group_right.move();
}
    

// currently this function takes a Pose from the camera, transforms it into robot frame
// and tells the left end-effector to move there
void left_callback(geometry_msgs::Pose msg) {
    moveit::planning_interface::MoveGroup group_left("arm_left");
    geometry_msgs::Pose target_pose_left;
    geometry_msgs::Pose currentPose;
    tf::Transform camTransform;
    tf::Vector3 camVector;
    tf::Vector3 goalVector;
    std::vector<double> currentRPY;

    // get current pose of end effector
    currentPose = group_left.getCurrentPose().pose;
    currentRPY = group_left.getCurrentRPY();

    // print human-readable orientation of end-effector
    std::cout << "rpy POSE x: " << currentRPY[0] << "\n";
    std::cout << "rpy POSE y: " << currentRPY[1] << "\n";
    std::cout << "rpy POSE z: " << currentRPY[2] << "\n\n";

    std::cout << "POSE x: " << currentPose.position.x << "\n";
    std::cout << "POSE y: " << currentPose.position.y << "\n";
    std::cout << "POSE z: " << currentPose.position.z << "\n";
    std::cout << "POSE rx: " << currentPose.orientation.x << "\n";
    std::cout << "POSE ry: " << currentPose.orientation.y << "\n";
    std::cout << "POSE rz: " << currentPose.orientation.z << "\n";
    std::cout << "POSE rw: " << currentPose.orientation.w << "\n\n";  
    
    // set quatMsg to orientation of object in camera frame
    Eigen::Quaterniond quatMsg(msg.orientation.w, msg.orientation.x, msg.orientation.y,     msg.orientation.z);
    
    // set quatWrist to orientation of end-effector
    Eigen::Quaterniond quatWrist(currentPose.orientation.w, currentPose.orientation.x,  currentPose.orientation.y, currentPose.orientation.z);
    
    // real world camera translation from end-effector    
    float camX = 0.0;
    float camY = 0.0;
    float camZ = 0.0;

    // real world camera orientation (Euler values) relative to end-effector
    float camRW = 1.0;
    float camRX = 0.0;
    float camRY = 0.0;
    float camRZ = 0.0;

    // set quatCam to orientation of camera in end-effector frame
    Eigen::Quaterniond quatCam(camRW, camRX, camRY, camRZ);

    Eigen::Quaterniond quatMsg(msg.orientation.w, msg.orientation.x, msg.orientation.y,     msg.orientation.z);
    
    // set camVector equal to translation from end-effector to object in end-effector frame    
    camVector.setValue(msg.position.x + camX, msg.position.y + camY, msg.position.z + camZ);
 
    std::cout << "camVector x: " << camVector.x() << "\n";
    std::cout << "camVector y: " << camVector.y() << "\n";
    std::cout << "camVector z: " << camVector.z() << "\n\n";
    
    // set camTransform rotation to orientation of end-effector    
    camTransform.setRotation(quatWrist);

    // transform camVector from end-effector frame into robot frame
    goalVector = camTransform(camVector);
    
    std::cout << "goalVector x: " << goalVector.x() << "\n";
    std::cout << "goalVector y: " << goalVector.y() << "\n";
    std::cout << "goalVector z: " << goalVector.z() << "\n\n";
    
    // set target position by adding goalVector to current position
    target_pose_left.position.x = goalVector.x() + currentPose.position.x;
    target_pose_left.position.y = goalVector.y() + currentPose.position.y;
    target_pose_left.position.z = goalVector.z() + currentPose.position.z;
    
    // initialize quatNew, set it to orientation of object in robot frame
    // note that this code assumes the camera and wrist have the same orientation   
    tf::Quaternion quatNew(0, 0, 0, 1);
    
    // quatNew is orientation of object (from camera msg) relative to orientation of end-effector
    quatNew = quatMsg * quatWrist;
        
    // set target orientation to orientation of object in robot frame
    target_pose_left.orientation.x = (double)quatNew.x();
    target_pose_left.orientation.y = (double)quatNew.y();
    target_pose_left.orientation.z = (double)quatNew.z();
    target_pose_left.orientation.w = (double)quatNew.w();
    
    std::cout << "Target POSE x: " << target_pose_left.position.x << "\n";
    std::cout << "Target POSE y: " << target_pose_left.position.y << "\n";
    std::cout << "Target POSE z: " << target_pose_left.position.z << "\n";
    std::cout << "Target POSE rx: " << target_pose_left.orientation.x << "\n";
    std::cout << "Target POSE ry: " << target_pose_left.orientation.y << "\n";
    std::cout << "Target POSE rz: " << target_pose_left.orientation.z << "\n";
    std::cout << "Target POSE rw: " << target_pose_left.orientation.w << "\n\n";

    // set target pose for left end effector    
    group_left.setPoseTarget(target_pose_left);
        
    // compute and visualize plan
    moveit::planning_interface::MoveGroup::Plan left_plan;
    bool success = group_left.plan(left_plan);
    
    // Sleep while plan is shown in Rviz
    sleep(5.0);
    
    // move to target pose
    group_left.move();    
}
    
int main(int argc, char **argv) {
    ros::init(argc, argv, "move_group_listen");
    ros::NodeHandle node_handle;  
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    // let Rviz start
    sleep(15.0);
    
    // wait for new target pose for either end-effector
    ros::Subscriber right_sub = node_handle.subscribe("/arm_right_target", 1, right_callback);
    ros::Subscriber left_sub = node_handle.subscribe("/arm_left_target", 1, left_callback);
    
    std::cout << "WAITING..." << "\n";
    
    
    while(1){}
    
}
