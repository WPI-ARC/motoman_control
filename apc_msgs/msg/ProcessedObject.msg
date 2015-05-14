int32 NO_OBJECTS_FOUND = -1
int32 INVALID_OBJECT = -2
int32 EMPTY_POINTCLOUD = -3
int32 SUCCESS = 0

sensor_msgs/PointCloud2 pointcloud # Target object pointcloud
sensor_msgs/PointCloud2 voxelcloud # Voxelized version of object pointcloud
geometry_msgs/Transform transfrom # transform for pointcloud 
geometry_msgs/Pose pose
geometry_msgs/Vector3 obb_extents
sensor_msgs/PointCloud2 collision_cloud # Pointcloud to be used for collision checking
int32 status
