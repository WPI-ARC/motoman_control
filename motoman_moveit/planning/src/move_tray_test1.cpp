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
#include <moveit/planning_scene/planning_scene.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_there_move_back");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::service::waitForService("move_group_service");
    std::cout << "SERVICES STARTED!" << std::endl;
    for (int i = 0; i < 11; i++) {
        sleep(1.0);
        std::cout << i << " seconds" << std::endl;
    }
    std::cout << "-------DONE SLEEPING----" << std::endl;

    int n = 1;
    while (1) {

        // move to pose 0
        moveit::planning_interface::MoveGroup move_group("arm_left");
        move_group.setPlannerId("RRTstarkConfigDefault");
        move_group.setPlanningTime(5.0);
        move_group.setStartStateToCurrentState();

        move_group.setGoalOrientationTolerance(0.001);
        move_group.setGoalPositionTolerance(0.0001);
        move_group.setGoalJointTolerance(0.001);

        std::vector<double> leftVector(7);

/*
        leftVector[0] = 0.0;
        leftVector[1] = 0.0;
        leftVector[2] = 0.0;
        leftVector[3] = 0.0;
        leftVector[4] = 0.0;
        leftVector[5] = 0.0;
        leftVector[6] = 0.0;



        move_group.setJointValueTarget(leftVector);

        moveit::planning_interface::MoveGroup::Plan plan;
        bool planSuccess = move_group.plan(plan);

        // slow down velocity
        int planSize = plan.trajectory_.joint_trajectory.points.size();
        for (int i = 0; i < planSize; i++) {
            plan.trajectory_.joint_trajectory.points[i].time_from_start *= n;
            for (int j = 0; j < 7; j++) {
                plan.trajectory_.joint_trajectory.points[i].velocities[j] *= 1/n;
            }
        }

        move_group.execute(plan);
        sleep(1.0);

        std::cout << "##########   MOVED TO POSE #0   ##########" << std::endl;


        // move to pose 0.5
        move_group.setPlanningTime(5.0);


        leftVector[0] = -0.162;
        leftVector[1] = 0.542;
        leftVector[2] = 1.853;
        leftVector[3] = -0.485;
        leftVector[4] = -0.887;
        leftVector[5] = 0.230;
        leftVector[6] = 0.637;

        move_group.setJointValueTarget(leftVector);

        moveit::planning_interface::MoveGroup::Plan plan3;
        move_group.setStartStateToCurrentState();
        bool planSuccess3 = move_group.plan(plan3);

        int planSize3 = plan3.trajectory_.joint_trajectory.points.size();
        for (int k = 0; k < planSize3; k++) {
            plan3.trajectory_.joint_trajectory.points[k].time_from_start *= n;
            for (int l = 0; l < 7; l++) {
                plan3.trajectory_.joint_trajectory.points[k].velocities[l] *= 1/n;
            }
        }

        move_group.execute(plan3);
        sleep(1.0);

        std::cout << "##########   MOVED TO POSE #0.5   ##########" << std::endl;
*/

        // move to pose 1
        move_group.setPlanningTime(5.0);


        leftVector[0] = 1.044;
        leftVector[1] = -0.988;
        leftVector[2] = 0.0;
        leftVector[3] = 1.686;
        leftVector[4] = 0.002;
        leftVector[5] = 0.888;
        leftVector[6] = -0.555;

        move_group.setJointValueTarget(leftVector);

        moveit::planning_interface::MoveGroup::Plan plan1;
        move_group.setStartStateToCurrentState();
        bool planSuccess1 = move_group.plan(plan1);

        int planSize1 = plan1.trajectory_.joint_trajectory.points.size();
        for (int k = 0; k < planSize1; k++) {
            plan1.trajectory_.joint_trajectory.points[k].time_from_start *= n;
            for (int l = 0; l < 7; l++) {
                plan1.trajectory_.joint_trajectory.points[k].velocities[l] *= 1/n;
            }
        }

        move_group.execute(plan1);
        sleep(1.0);

        std::cout << "##########   MOVED TO POSE #1   ##########" << std::endl;


        // move to pose 2
        geometry_msgs::Pose poseTarget;

        /*
        poseTarget.position.x = 0.28805;
        poseTarget.position.y = -0.043939;
        poseTarget.position.z = 1.7018;
        poseTarget.orientation.x = 0.028426;
        poseTarget.orientation.y = 0.7007;
        poseTarget.orientation.z = 0.70982;
        poseTarget.orientation.w = -0.066036;*/

        poseTarget.position.x = 0.33373;
        poseTarget.position.y = -0.040;
        poseTarget.position.z = 1.6985;
        poseTarget.orientation.x = 0.042959;
        poseTarget.orientation.y = 0.70606;
        poseTarget.orientation.z = 0.70488;
        poseTarget.orientation.w = -0.052714;

        moveit::planning_interface::MoveGroup::Plan plan2;
        move_group.setStartStateToCurrentState();
        move_group.setPoseTarget(poseTarget);
        bool planSuccess2 = move_group.plan(plan2);


        move_group.execute(plan2);
        sleep(1.0);

        std::cout << "##########   MOVED TO POSE #2   ##########" << std::endl;


        // move to pose 3
        std::vector<geometry_msgs::Pose> poseVector(2);

        poseVector[0].position.x = 0.33373;
        poseVector[0].position.y = -0.040;
        poseVector[0].position.z = 1.6985;
        poseVector[0].orientation.x = 0.042959;
        poseVector[0].orientation.y = 0.70606;
        poseVector[0].orientation.z = 0.70488;
        poseVector[0].orientation.w = -0.052714;

        poseVector[1].position.x = 0.44;
        poseVector[1].position.y = -0.044042;
        poseVector[1].position.z = 1.65;
        poseVector[1].orientation.x = 0.042905;
        poseVector[1].orientation.y = 0.70623;
        poseVector[1].orientation.z = 0.70471;
        poseVector[1].orientation.w = -0.052789;

        moveit::planning_interface::MoveGroup::Plan plan3;
        move_group.setStartStateToCurrentState();
        //bool planSuccess3 = move_group.plan(plan4);
        move_group.computeCartesianPath(poseVector, 0.01, 100, plan3.trajectory_, false);

        move_group.execute(plan3);
        sleep(1.0);

        std::cout << "##########   MOVED TO POSE #3   ##########" << std::endl;

        // move to pose 4
        poseVector[0].position.x = 0.44;
        poseVector[0].position.y = -0.044042;
        poseVector[0].position.z = 1.65;
        poseVector[0].orientation.x = 0.042905;
        poseVector[0].orientation.y = 0.70623;
        poseVector[0].orientation.z = 0.70471;
        poseVector[0].orientation.w = -0.052789;

        poseVector[1].position.x = 0.44;
        poseVector[1].position.y = -0.044003;
        poseVector[1].position.z = 1.4876;
        poseVector[1].orientation.x = 0.042921;
        poseVector[1].orientation.y = 0.70748;
        poseVector[1].orientation.z = 0.70345;
        poseVector[1].orientation.w = -0.052767;

        moveit::planning_interface::MoveGroup::Plan plan4;
        move_group.setStartStateToCurrentState();
        move_group.computeCartesianPath(poseVector, 0.01, 100, plan4.trajectory_, false);

        move_group.execute(plan4);
        sleep(1.0);

        std::cout << "##########   MOVED TO POSE #4   ##########" << std::endl;


        // move to pose 5
        poseVector[0].position.x = 0.43446;
        poseVector[0].position.y = -0.044003;
        poseVector[0].position.z = 1.4876;
        poseVector[0].orientation.x = 0.042921;
        poseVector[0].orientation.y = 0.70748;
        poseVector[0].orientation.z = 0.70345;
        poseVector[0].orientation.w = -0.052767;

        poseVector[1].position.x = 0.75199;
        poseVector[1].position.y = -0.040048;
        poseVector[1].position.z = 1.4877;
        poseVector[1].orientation.x = 0.042761;
        poseVector[1].orientation.y = 0.70764;
        poseVector[1].orientation.z = 0.70329;
        poseVector[1].orientation.w = -0.052929;

        moveit::planning_interface::MoveGroup::Plan plan5;
        move_group.setStartStateToCurrentState();
        move_group.computeCartesianPath(poseVector, 0.01, 100, plan5.trajectory_, false);

        move_group.execute(plan5);
        sleep(1.0);

        std::cout << "##########   MOVED TO POSE #5   ##########" << std::endl;


        // move to pose 6
        poseVector[0].position.x = 0.75199;
        poseVector[0].position.y = -0.040048;
        poseVector[0].position.z = 1.4877;
        poseVector[0].orientation.x = 0.042761;
        poseVector[0].orientation.y = 0.70764;
        poseVector[0].orientation.z = 0.70329;
        poseVector[0].orientation.w = -0.052929;

        poseVector[1].position.x = 0.73266;
        poseVector[1].position.y = -0.040059;
        poseVector[1].position.z = 1.4303;
        poseVector[1].orientation.x = 0.091728;
        poseVector[1].orientation.y = 0.70223;
        poseVector[1].orientation.z = 0.6986;
        poseVector[1].orientation.w = -0.10205;

        moveit::planning_interface::MoveGroup::Plan plan6;
        move_group.setStartStateToCurrentState();
        move_group.computeCartesianPath(poseVector, 0.01, 100, plan6.trajectory_, false);

        move_group.execute(plan6);
        sleep(1.0);

        std::cout << "##########   MOVED TO POSE #6   ##########" << std::endl;


        // move to pose 7
        poseVector[0].position.x = 0.73266;
        poseVector[0].position.y = -0.040059;
        poseVector[0].position.z = 1.4303;
        poseVector[0].orientation.x = 0.091728;
        poseVector[0].orientation.y = 0.70223;
        poseVector[0].orientation.z = 0.6986;
        poseVector[0].orientation.w = -0.10205;

        poseVector[1].position.x = 0.70613;
        poseVector[1].position.y = -0.04086;
        poseVector[1].position.z = 1.3885;
        poseVector[1].orientation.x = 0.091678;
        poseVector[1].orientation.y = 0.70225;
        poseVector[1].orientation.z = 0.69859;
        poseVector[1].orientation.w = -0.10205;

        moveit::planning_interface::MoveGroup::Plan plan7;
        move_group.setStartStateToCurrentState();
        move_group.computeCartesianPath(poseVector, 0.01, 100, plan7.trajectory_, false);

        move_group.execute(plan7);
        sleep(1.0);

        std::cout << "##########   MOVED TO POSE #7   ##########" << std::endl;

        // move to pose 8
        poseVector[0].position.x = 0.70613;
        poseVector[0].position.y = -0.04086;
        poseVector[0].position.z = 1.3885;
        poseVector[0].orientation.x = 0.091678;
        poseVector[0].orientation.y = 0.70225;
        poseVector[0].orientation.z = 0.69859;
        poseVector[0].orientation.w = -0.10205;

        poseVector[1].position.x = 0.42909;
        poseVector[1].position.y = -0.04087;
        poseVector[1].position.z = 1.2856;
        poseVector[1].orientation.x = -0.26666;
        poseVector[1].orientation.y = 0.65845;
        poseVector[1].orientation.z = 0.65426;
        poseVector[1].orientation.w = 0.25939;

        moveit::planning_interface::MoveGroup::Plan plan8;
        move_group.setStartStateToCurrentState();
        move_group.computeCartesianPath(poseVector, 0.01, 100, plan8.trajectory_, false);

        move_group.execute(plan8);
        sleep(1.0);

        std::cout << "##########   MOVED TO POSE #8   ##########" << std::endl;

/*
        // move to pose 3
        move_group.setPlanningTime(5.0);

        poseVector[0].position.x = 0.28805;
        poseVector[0].position.y = -0.043939;
        poseVector[0].position.z = 1.7018;
        poseVector[0].orientation.x = 0.028426;
        poseVector[0].orientation.y = 0.7007;
        poseVector[0].orientation.z = 0.70982;
        poseVector[0].orientation.w = -0.066036;

        poseVector[1].position.x = 0.41444;
        poseVector[1].position.y = -0.044031;
        poseVector[1].position.z = 1.6992;
        poseVector[1].orientation.x = 0.028394;
        poseVector[1].orientation.y = 0.70085;
        poseVector[1].orientation.z = 0.70967;
        poseVector[1].orientation.w = -0.066099;

        moveit::planning_interface::MoveGroup::Plan plan3;
        move_group.setStartStateToCurrentState();
        //bool planSuccess3 = move_group.plan(plan4);
        move_group.computeCartesianPath(poseVector, 0.01, 100, plan3.trajectory_, false);

        move_group.execute(plan3);
        sleep(1.0);

        std::cout << "##########   MOVED TO POSE #3   ##########" << std::endl;


        // move to pose 3
        move_group.setPlanningTime(5.0);

        poseVector[0].position.x = 0.28805;
        poseVector[0].position.y = -0.043939;
        poseVector[0].position.z = 1.7018;
        poseVector[0].orientation.x = 0.028426;
        poseVector[0].orientation.y = 0.7007;
        poseVector[0].orientation.z = 0.70982;
        poseVector[0].orientation.w = -0.066036;

        poseVector[1].position.x = 0.41444;
        poseVector[1].position.y = -0.044031;
        poseVector[1].position.z = 1.6992;
        poseVector[1].orientation.x = 0.028394;
        poseVector[1].orientation.y = 0.70085;
        poseVector[1].orientation.z = 0.70967;
        poseVector[1].orientation.w = -0.066099;

        moveit::planning_interface::MoveGroup::Plan plan3;
        move_group.setStartStateToCurrentState();
        //bool planSuccess3 = move_group.plan(plan4);
        move_group.computeCartesianPath(poseVector, 0.01, 100, plan3.trajectory_, false);

        move_group.execute(plan3);
        sleep(1.0);

        std::cout << "##########   MOVED TO POSE #3   ##########" << std::endl;


        // move to pose 3
        move_group.setPlanningTime(5.0);

        poseVector[0].position.x = 0.28805;
        poseVector[0].position.y = -0.043939;
        poseVector[0].position.z = 1.7018;
        poseVector[0].orientation.x = 0.028426;
        poseVector[0].orientation.y = 0.7007;
        poseVector[0].orientation.z = 0.70982;
        poseVector[0].orientation.w = -0.066036;

        poseVector[1].position.x = 0.41444;
        poseVector[1].position.y = -0.044031;
        poseVector[1].position.z = 1.6992;
        poseVector[1].orientation.x = 0.028394;
        poseVector[1].orientation.y = 0.70085;
        poseVector[1].orientation.z = 0.70967;
        poseVector[1].orientation.w = -0.066099;

        moveit::planning_interface::MoveGroup::Plan plan3;
        move_group.setStartStateToCurrentState();
        //bool planSuccess3 = move_group.plan(plan4);
        move_group.computeCartesianPath(poseVector, 0.01, 100, plan3.trajectory_, false);

        move_group.execute(plan3);
        sleep(1.0);

        std::cout << "##########   MOVED TO POSE #3   ##########" << std::endl;


        // move to pose 3
        move_group.setPlanningTime(5.0);

        poseVector[0].position.x = 0.28805;
        poseVector[0].position.y = -0.043939;
        poseVector[0].position.z = 1.7018;
        poseVector[0].orientation.x = 0.028426;
        poseVector[0].orientation.y = 0.7007;
        poseVector[0].orientation.z = 0.70982;
        poseVector[0].orientation.w = -0.066036;

        poseVector[1].position.x = 0.41444;
        poseVector[1].position.y = -0.044031;
        poseVector[1].position.z = 1.6992;
        poseVector[1].orientation.x = 0.028394;
        poseVector[1].orientation.y = 0.70085;
        poseVector[1].orientation.z = 0.70967;
        poseVector[1].orientation.w = -0.066099;

        moveit::planning_interface::MoveGroup::Plan plan3;
        move_group.setStartStateToCurrentState();
        //bool planSuccess3 = move_group.plan(plan4);
        move_group.computeCartesianPath(poseVector, 0.01, 100, plan3.trajectory_, false);

        move_group.execute(plan3);
        sleep(1.0);

        std::cout << "##########   MOVED TO POSE #3   ##########" << std::endl;


        // move to pose 3
        move_group.setPlanningTime(5.0);

        poseVector[0].position.x = 0.28805;
        poseVector[0].position.y = -0.043939;
        poseVector[0].position.z = 1.7018;
        poseVector[0].orientation.x = 0.028426;
        poseVector[0].orientation.y = 0.7007;
        poseVector[0].orientation.z = 0.70982;
        poseVector[0].orientation.w = -0.066036;

        poseVector[1].position.x = 0.41444;
        poseVector[1].position.y = -0.044031;
        poseVector[1].position.z = 1.6992;
        poseVector[1].orientation.x = 0.028394;
        poseVector[1].orientation.y = 0.70085;
        poseVector[1].orientation.z = 0.70967;
        poseVector[1].orientation.w = -0.066099;

        moveit::planning_interface::MoveGroup::Plan plan3;
        move_group.setStartStateToCurrentState();
        //bool planSuccess3 = move_group.plan(plan4);
        move_group.computeCartesianPath(poseVector, 0.01, 100, plan3.trajectory_, false);

        move_group.execute(plan3);
        sleep(1.0);

        std::cout << "##########   MOVED TO POSE #3   ##########" << std::endl;


        // move to pose 3
        move_group.setPlanningTime(5.0);

        poseVector[0].position.x = 0.28805;
        poseVector[0].position.y = -0.043939;
        poseVector[0].position.z = 1.7018;
        poseVector[0].orientation.x = 0.028426;
        poseVector[0].orientation.y = 0.7007;
        poseVector[0].orientation.z = 0.70982;
        poseVector[0].orientation.w = -0.066036;

        poseVector[1].position.x = 0.41444;
        poseVector[1].position.y = -0.044031;
        poseVector[1].position.z = 1.6992;
        poseVector[1].orientation.x = 0.028394;
        poseVector[1].orientation.y = 0.70085;
        poseVector[1].orientation.z = 0.70967;
        poseVector[1].orientation.w = -0.066099;

        moveit::planning_interface::MoveGroup::Plan plan3;
        move_group.setStartStateToCurrentState();
        //bool planSuccess3 = move_group.plan(plan4);
        move_group.computeCartesianPath(poseVector, 0.01, 100, plan3.trajectory_, false);

        move_group.execute(plan3);
        sleep(1.0);

        std::cout << "##########   MOVED TO POSE #3   ##########" << std::endl;


        // move to pose 3
        move_group.setPlanningTime(5.0);

        poseVector[0].position.x = 0.28805;
        poseVector[0].position.y = -0.043939;
        poseVector[0].position.z = 1.7018;
        poseVector[0].orientation.x = 0.028426;
        poseVector[0].orientation.y = 0.7007;
        poseVector[0].orientation.z = 0.70982;
        poseVector[0].orientation.w = -0.066036;

        poseVector[1].position.x = 0.41444;
        poseVector[1].position.y = -0.044031;
        poseVector[1].position.z = 1.6992;
        poseVector[1].orientation.x = 0.028394;
        poseVector[1].orientation.y = 0.70085;
        poseVector[1].orientation.z = 0.70967;
        poseVector[1].orientation.w = -0.066099;

        moveit::planning_interface::MoveGroup::Plan plan3;
        move_group.setStartStateToCurrentState();
        //bool planSuccess3 = move_group.plan(plan4);
        move_group.computeCartesianPath(poseVector, 0.01, 100, plan3.trajectory_, false);

        move_group.execute(plan3);
        sleep(1.0);

        std::cout << "##########   MOVED TO POSE #3   ##########" << std::endl;


        // move to pose 3
        move_group.setPlanningTime(5.0);

        poseVector[0].position.x = 0.28805;
        poseVector[0].position.y = -0.043939;
        poseVector[0].position.z = 1.7018;
        poseVector[0].orientation.x = 0.028426;
        poseVector[0].orientation.y = 0.7007;
        poseVector[0].orientation.z = 0.70982;
        poseVector[0].orientation.w = -0.066036;

        poseVector[1].position.x = 0.41444;
        poseVector[1].position.y = -0.044031;
        poseVector[1].position.z = 1.6992;
        poseVector[1].orientation.x = 0.028394;
        poseVector[1].orientation.y = 0.70085;
        poseVector[1].orientation.z = 0.70967;
        poseVector[1].orientation.w = -0.066099;

        moveit::planning_interface::MoveGroup::Plan plan3;
        move_group.setStartStateToCurrentState();
        //bool planSuccess3 = move_group.plan(plan4);
        move_group.computeCartesianPath(poseVector, 0.01, 100, plan3.trajectory_, false);

        move_group.execute(plan3);
        sleep(1.0);

        std::cout << "##########   MOVED TO POSE #3   ##########" << std::endl;
*/
        std::cout << "##########   DONE   ##########" << std::endl;
        return true;
    }

}
