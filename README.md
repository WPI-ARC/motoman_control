# motoman_control
Control software for the motoman robot

I updated the master branch to a basic functional workspace with a custom package called motoman_moveit.

-bring up the motoman Rviz demo with subscribers waiting for a new target Pose for either end effector:

roslaunch motoman_moveit motoman_group_listen.launch

-publish hard-coded Pose to chosen end effector:

roslaunch motoman_moveit motoman_group_target_pose.launch
