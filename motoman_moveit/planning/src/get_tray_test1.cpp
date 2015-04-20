#include <ros/ros.h>
#include <iostream>
#include <motoman_moveit/get_tray_server.h>
//#include <motoman_moveit/attach_tray_server.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "get_tray_test1");
    ros::NodeHandle nh;

    ros::service::waitForService("get_tray_service");
    ros::service::waitForService("convert_trajectory_service");
    //ros::service::waitForService("attach_tray_service");

    std::cout << "DONE WAITING FOR SERVICES TO START" << std::endl;

    motoman_moveit::get_tray_server move1;
    move1.request.hand = "left";
    ros::service::call("get_tray_service", move1);

    /*
    motoman_moveit::attach_tray_server attach1;
    attach1.hand = "left";
    ros::service::call("attach_tray_service", attach1);
    */

}
