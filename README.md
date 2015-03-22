# motoman_control
Control software for the motoman robot

## Basic Control

I updated the master branch to a basic functional workspace with a
custom package called motoman_moveit.

1. Bring up the motoman Rviz demo with subscribers waiting for a new
   target Pose for either end effector:
   - `roslaunch motoman_moveit motoman_group_listen.launch`
2. Publish hard-coded Pose to chosen end effector:
   - `roslaunch motoman_moveit motoman_group_target_pose.launch`

## Task Controller

The task controller contains a state machine that runs the Amazon
Picking Challenge pick and place code.

1. To run it, first launch the demo:
   - `roslaunch task_controller main.launch`
2. (Optional) If you want to view the state machine, run:
   - `rosrun smach_viewer smach_viewer.py`
3. Run the task controller:
   - `rosrun task_controller motoman_controller.py`
