#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "ros/ros.h"
#include "apc_util/PublishPointcloudCollision.h"

#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

static ros::Publisher pub;

bool callback(apc_util::PublishPointcloudCollision::Request  &req,
              apc_util::PublishPointcloudCollision::Response &res)
{
  ROS_INFO("request: ");

  // Convert pointcloud
  pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(req.pointcloud, *cloud);

  // Radius filter
  pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> outrem;
  pcl::PCLPointCloud2::Ptr cloud_radius_filtered(new pcl::PCLPointCloud2);
  outrem.setInputCloud(cloud);
  outrem.setRadiusSearch(0.025);
  outrem.setMinNeighborsInRadius(2);
  outrem.filter(*cloud_radius_filtered);

  std::cerr << "PointCloud after radius filtering: " << cloud_radius_filtered->width * cloud_radius_filtered->height 
            << " data points (" << pcl::getFieldsList(*cloud_radius_filtered) << ").\n";

  
  // Voxel Filter 
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);
  sor.setInputCloud(cloud_radius_filtered);
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  sor.filter(*cloud_filtered);

  std::cerr << "PointCloud after voxel filtering: " << cloud_filtered->width * cloud_filtered->height 
            << " data points (" << pcl::getFieldsList(*cloud_filtered) << ").\n";

  // Publish Collision Objects

  PointCloud::Ptr voxel_cloud(new PointCloud);
  pcl::fromPCLPointCloud2(*cloud_filtered, *voxel_cloud);
  
  shape_msgs::SolidPrimitive voxel;
  voxel.type = shape_msgs::SolidPrimitive::BOX;
  voxel.dimensions.push_back(0.01);
  voxel.dimensions.push_back(0.01);
  voxel.dimensions.push_back(0.01);

  geometry_msgs::Pose pose;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 0;
  
  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = req.pointcloud.header.frame_id;
  co.id = "pointcloud_voxels";
  co.operation = moveit_msgs::CollisionObject::ADD;
  for (unsigned int i = 0; i < (*voxel_cloud).points.size(); i++) {
    co.primitives.push_back(voxel);
    pose.position.x = (*voxel_cloud).points[i].x;
    pose.position.y = (*voxel_cloud).points[i].y;
    pose.position.z = (*voxel_cloud).points[i].z;
    co.primitive_poses.push_back(pose);
  }
  pub.publish(co);
  
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publish_pointcloud_collision_server");
  ros::NodeHandle n;
  pub = n.advertise<moveit_msgs::CollisionObject>("/collision_object", 1000);

  ros::ServiceServer service = n.advertiseService("publish_pointcloud_collision", callback);
  ROS_INFO("publish_pointcloud_collision_server ready.");
  ros::spin();

  return 0;
}
