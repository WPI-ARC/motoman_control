import rospy
import smach

from copy import deepcopy
# from geometry_msgs.msg import Pose, Point, Quaternion
from motoman_moveit.srv import convert_trajectory_server

from apc_util.moveit import follow_path, goto_pose, execute_known_trajectory
from apc_util.shelf import bin_pose, add_shelf, remove_shelf, Shelf
from apc_util.smach import on_exception


class Scoop(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['bin'], output_keys=[])

        self.arm = robot.arm_right_torso
        self.move = rospy.ServiceProxy("/convert_trajectory_service",
                                       convert_trajectory_server)

        self.rightColumn = False

    @on_exception(failure_state="Failure")
    def execute(self, userdata):
        # REMOVE THIS UNLESS 'PushWithScoop' IS CALLING EVERYTHING
        # return 'Success'

        jointValues = [0, 0, 0, 0, 0, 0, 0, 0]
        
        if userdata.bin == "A":
            jointValues = [2.608074188232422, -0.29658669233322144, 0.8934586644172668, 1.7289633750915527, 1.573803424835205, 1.2867212295532227, 1.4699939489364624, -2.8265552520751953]

        elif userdata.bin == "B":
            pass
            # jointValues = 

        elif userdata.bin == "C":
            pass
            # jointValues = 

        elif userdata.bin == "D":
            jointValues = [2.9667019844055176, 2.7412068843841553, 0.09522612392902374, -1.1803146600723267, 2.2825026512145996, 1.8705755472183228, 1.8874949216842651, -2.919917583465576]

        elif userdata.bin == "E":
            jointValues = [2.9667019844055176, 1.4224945306777954, -0.7801656126976013, -0.2995363175868988, 2.195582151412964, 1.864424467086792, 1.6602683067321777, 2.5383474826812744]

        elif userdata.bin == "F":
            jointValues = [1.5194422006607056,1.810523509979248, -1.2088792324066162, 1.3328773975372314, -1.8696491718292236, 1.8829082250595093, -1.2678426504135132, 1.606799840927124]

        elif userdata.bin == "G":
            pass
            # jointValues = 

        elif userdata.bin == "H":
            jointValues = [2.966156482696533, 1.8770301342010498, 1.2306787967681885, -0.586269199848175, 2.2546935081481934, 1.669684886932373, 1.7160991430282593, 0.7149554491043091]

        elif userdata.bin == "I":
            jointValues = [1.5194591283798218, 1.251114845275879, -1.8047455549240112, 2.224393606185913, -1.9810069799423218, 1.1204286813735962, -1.827457070350647, 0.8016403913497925]

        elif userdata.bin == "J":
            jointValues = [2.608074188232422, 0.4578932821750641, 1.8810696601867676, -0.5525216460227966, 1.9467278718948364, 0.23977181315422058, 0.7547944784164429, -0.43715447187423706]

        elif userdata.bin == "K":
            jointValues = [2.9667019844055176, -0.873210072517395, -0.5380352735519409, 2.7276151180267334, -2.2068514823913574, 1.085071086883545, 1.8169622421264648, 1.6070705652236938]

        elif userdata.bin == "L":
            jointValues = [1.7551809549331665, 0.04665006324648857, -1.8453619480133057, 1.8693605661392212, -1.189427375793457, 1.5698546171188354, -1.871213436126709, 0.8811066150665283]



        rospy.loginfo("Trying to scoop from bin '"+userdata.bin+"' ")

        # SCOOP
        self.arm.set_workspace([-3, -3, -3, 3, 3, 3])
        self.arm.set_planning_time(10)
        self.arm.set_planner_id("RRTConnectkConfigDefault")
        self.arm.set_pose_reference_frame("/shelf")

        # Pose relative to bin used for robustness
        horizontalPose = bin_pose(userdata.bin).pose
        horizontalPose.position.x += -0.307581
        horizontalPose.position.y += -0.011221
        horizontalPose.position.z += 0.05463
        horizontalPose.orientation.x = -0.293106
        horizontalPose.orientation.y = -0.512959
        horizontalPose.orientation.z = 0.403541
        horizontalPose.orientation.w = 0.698654

        leftOffset = -0.001
        middleOffset = -0.002
        rightOffset = 0.050

        if (userdata.bin == "A" or userdata.bin == "D" or
                userdata.bin == "G" or userdata.bin == "J"):
            horizontalPose.position.y += leftOffset

        elif (userdata.bin == "B" or userdata.bin == "E" or
                userdata.bin == "H" or userdata.bin == "K"):
            horizontalPose.position.y += middleOffset

        elif (userdata.bin == "C" or userdata.bin == "F" or
                userdata.bin == "I" or userdata.bin == "L"):
            horizontalPose.position.y += rightOffset
            self.rightColumn = True



        horizontalPose = self.convertFrameRobotToShelf(horizontalPose)
        rospy.loginfo("planning to horizontal pose")

        self.arm.set_planning_time(5)
        self.arm.set_joint_value_target(jointValues)
        remove_shelf()  # SHELF SHOULD NOT ACTUALLY BE REMOVED
        plan = self.arm.plan()
        if not self.move(plan.joint_trajectory):
            return 'Failure'

        # self.arm.set_pose_target(horizontalPose)
        # remove_shelf()  # SHELF SHOULD NOT ACTUALLY BE REMOVED
        # plan = self.arm.plan()
        # self.move(plan.joint_trajectory)

        # add_shelf()
        # poses = [self.convertFrameRobotToShelf(self.arm.
        #                                        get_current_pose().pose)]
        # poses.append(deepcopy(poses[-1]))
        # # poses[-1].position.x += 0.02
        # poses[-1].orientation.x = -0.50011
        # poses[-1].orientation.y = 0.52604
        # poses[-1].orientation.z = 0.49273
        # poses[-1].orientation.w = -0.47999

        # # if not follow_path(self.arm, poses):
        # #     return 'Failure'

        # # poses = [self.convertFrameRobotToShelf(self.arm.
        # #                                        get_current_pose().pose)]

        # poses.append(deepcopy(poses[-1]))
        # # poses[-1].position.x += -0.02
        # poses[-1].orientation.x = -0.69119
        # poses[-1].orientation.y = 0.043994
        # poses[-1].orientation.z = 0.72117
        # poses[-1].orientation.w = -0.015311

        # # if not follow_path(self.arm, poses):
        # #     return 'Failure'

        # # poses = [self.convertFrameRobotToShelf(self.arm.
        #                                        # get_current_pose().pose)]

        # poses.append(deepcopy(poses[-1]))
        # # poses[-1].position.x += 0.02
        # poses[-1].orientation.x = horizontalPose.orientation.x
        # poses[-1].orientation.y = horizontalPose.orientation.y
        # poses[-1].orientation.z = horizontalPose.orientation.z
        # poses[-1].orientation.w = horizontalPose.orientation.w

        # # if not follow_path(self.arm, poses):
        # #     return 'Failure'

        # # poses = [self.convertFrameRobotToShelf(self.arm.
        # #                                        get_current_pose().pose)]

        # poses.append(horizontalPose)
        # # if not follow_path(self.arm, poses):
        # #     return 'Failure'

        # traj, success = self.arm.compute_cartesian_path(
        #     poses,
        #     0.01,  # 1cm interpolation resolution
        #     0.0,  # jump_threshold disabled
        # )

        # if success < 1:
        #     rospy.logerr(
        #         "Cartesian trajectory could not be completed. Only solved for: '"
        #         + str(success) + "'..."
        #     )
        #     return 'Failure'

        # points = traj.joint_trajectory.points
        # speedRatio = 3.0

        # for point in points:
        #     for vel in points.velocities:
        #         vel /= speedRatio
        #     point.time_from_start *= speedRatio


        # if not self.move(traj.joint_trajectory):
        #     return 'Failure'

        remove_shelf()
        if not execute_known_trajectory(self.arm, 'Rotate', userdata.bin):
            return 'Failure'
        
        self.scoopBin(horizontalPose)

        rospy.loginfo("planning cartesian path out of bin")

        if userdata.bin == "A":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path out of bin")
            if not follow_path(self.arm, poses):
                return 'Failure'

            poses = [self.convertFrameRobotToShelf(self.arm.
                                                   get_current_pose().pose)]

            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path to final bin pose")
            if not follow_path(self.arm, poses):
                return 'Failure'

            poses = [self.convertFrameRobotToShelf(self.arm.
                                                   get_current_pose().pose)]            

            poses.append(deepcopy(poses[-1]))
            # To right side of shelf
            poses[-1].position.y = -0.686495  # INCLUDES CURRENT SHELF CALIBRATION as of Saturday night
            poses[-1].position.z += -0.05
            poses[-1].orientation.x = -0.36667
            poses[-1].orientation.y = -0.648119
            poses[-1].orientation.z = 0.333549
            poses[-1].orientation.w = 0.578135

            rospy.loginfo("planning cartesian path to pre-dumping pose")
            if not follow_path(self.arm, poses):
                return 'Failure'

        elif userdata.bin == "B":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path out of bin")
            if not follow_path(self.arm, poses):
                return 'Failure'

            poses = [self.convertFrameRobotToShelf(self.arm.
                                                   get_current_pose().pose)]

            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path to final bin pose")
            if not follow_path(self.arm, poses):
                return 'Failure'

            poses = [self.convertFrameRobotToShelf(self.arm.
                                                   get_current_pose().pose)]            

            poses.append(deepcopy(poses[-1]))
            # To right side of shelf
            poses[-1].position.y = -0.686495  # INCLUDES CURRENT SHELF CALIBRATION as of Saturday night
            poses[-1].position.z += -0.05
            poses[-1].orientation.x = -0.36667
            poses[-1].orientation.y = -0.648119
            poses[-1].orientation.z = 0.333549
            poses[-1].orientation.w = 0.578135

            rospy.loginfo("planning cartesian path to pre-dumping pose")
            if not follow_path(self.arm, poses):
                return 'Failure'

        elif userdata.bin == "C":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path out of bin")
            if not follow_path(self.arm, poses):
                return 'Failure'

            poses = [self.convertFrameRobotToShelf(self.arm.
                                                   get_current_pose().pose)]

            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path to final bin pose")
            if not follow_path(self.arm, poses):
                return 'Failure'

            poses = [self.convertFrameRobotToShelf(self.arm.
                                                   get_current_pose().pose)]            

            poses.append(deepcopy(poses[-1]))
            # To right side of shelf
            poses[-1].position.y = -0.686495  # INCLUDES CURRENT SHELF CALIBRATION as of Saturday night
            poses[-1].position.z += -0.05
            poses[-1].orientation.x = -0.36667
            poses[-1].orientation.y = -0.648119
            poses[-1].orientation.z = 0.333549
            poses[-1].orientation.w = 0.578135

            rospy.loginfo("planning cartesian path to pre-dumping pose")
            if not follow_path(self.arm, poses):
                return 'Failure'

        elif userdata.bin == "D":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path out of bin")
            if not follow_path(self.arm, poses):
                return 'Failure'

            poses = [self.convertFrameRobotToShelf(self.arm.
                                                   get_current_pose().pose)]

            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path to final bin pose")
            if not follow_path(self.arm, poses):
                return 'Failure'

            poses = [self.convertFrameRobotToShelf(self.arm.
                                                   get_current_pose().pose)]            

            poses.append(deepcopy(poses[-1]))
            # To right side of shelf
            poses[-1].position.y = -0.686495  # INCLUDES CURRENT SHELF CALIBRATION as of Saturday night
            poses[-1].position.z += -0.05
            poses[-1].orientation.x = -0.36667
            poses[-1].orientation.y = -0.648119
            poses[-1].orientation.z = 0.333549
            poses[-1].orientation.w = 0.578135

            rospy.loginfo("planning cartesian path to pre-dumping pose")
            if not follow_path(self.arm, poses):
                return 'Failure'

        elif userdata.bin == "E":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path out of bin")
            if not follow_path(self.arm, poses):
                return 'Failure'

            poses = [self.convertFrameRobotToShelf(self.arm.
                                                   get_current_pose().pose)]

            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path to final bin pose")
            if not follow_path(self.arm, poses):
                return 'Failure'

            poses = [self.convertFrameRobotToShelf(self.arm.
                                                   get_current_pose().pose)]            

            poses.append(deepcopy(poses[-1]))
            # To right side of shelf
            poses[-1].position.y = -0.686495  # INCLUDES CURRENT SHELF CALIBRATION as of Saturday night
            poses[-1].position.z += -0.05
            poses[-1].orientation.x = -0.36667
            poses[-1].orientation.y = -0.648119
            poses[-1].orientation.z = 0.333549
            poses[-1].orientation.w = 0.578135

            rospy.loginfo("planning cartesian path to pre-dumping pose")
            if not follow_path(self.arm, poses):
                return 'Failure'

        elif userdata.bin == "F":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path out of bin")
            if not follow_path(self.arm, poses):
                return 'Failure'

            poses = [self.convertFrameRobotToShelf(self.arm.
                                                   get_current_pose().pose)]

            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path to final bin pose")
            if not follow_path(self.arm, poses):
                return 'Failure'

            poses = [self.convertFrameRobotToShelf(self.arm.
                                                   get_current_pose().pose)]            

            poses.append(deepcopy(poses[-1]))
            # To right side of shelf
            poses[-1].position.y = -0.686495  # INCLUDES CURRENT SHELF CALIBRATION as of Saturday night
            poses[-1].position.z += -0.05
            poses[-1].orientation.x = -0.36667
            poses[-1].orientation.y = -0.648119
            poses[-1].orientation.z = 0.333549
            poses[-1].orientation.w = 0.578135

            rospy.loginfo("planning cartesian path to pre-dumping pose")
            if not follow_path(self.arm, poses):
                return 'Failure'

        elif userdata.bin == "G":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path out of bin")
            if not follow_path(self.arm, poses):
                return 'Failure'

            poses = [self.convertFrameRobotToShelf(self.arm.
                                                   get_current_pose().pose)]

            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path to final bin pose")
            if not follow_path(self.arm, poses):
                return 'Failure'

            poses = [self.convertFrameRobotToShelf(self.arm.
                                                   get_current_pose().pose)]            

            poses.append(deepcopy(poses[-1]))
            # To right side of shelf
            poses[-1].position.y = -0.686495  # INCLUDES CURRENT SHELF CALIBRATION as of Saturday night
            poses[-1].position.z += -0.05
            poses[-1].orientation.x = -0.36667
            poses[-1].orientation.y = -0.648119
            poses[-1].orientation.z = 0.333549
            poses[-1].orientation.w = 0.578135

            rospy.loginfo("planning cartesian path to pre-dumping pose")
            if not follow_path(self.arm, poses):
                return 'Failure'

        elif userdata.bin == "H":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path out of bin")
            if not follow_path(self.arm, poses):
                return 'Failure'

            poses = [self.convertFrameRobotToShelf(self.arm.
                                                   get_current_pose().pose)]

            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path to final bin pose")
            if not follow_path(self.arm, poses):
                return 'Failure'

            poses = [self.convertFrameRobotToShelf(self.arm.
                                                   get_current_pose().pose)]            

            poses.append(deepcopy(poses[-1]))
            # To right side of shelf
            poses[-1].position.y = -0.686495  # INCLUDES CURRENT SHELF CALIBRATION as of Saturday night
            poses[-1].position.z += -0.05
            poses[-1].orientation.x = -0.36667
            poses[-1].orientation.y = -0.648119
            poses[-1].orientation.z = 0.333549
            poses[-1].orientation.w = 0.578135

            rospy.loginfo("planning cartesian path to pre-dumping pose")
            if not follow_path(self.arm, poses):
                return 'Failure'

        elif userdata.bin == "I":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path out of bin")
            if not follow_path(self.arm, poses):
                return 'Failure'

            poses = [self.convertFrameRobotToShelf(self.arm.
                                                   get_current_pose().pose)]

            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path to final bin pose")
            if not follow_path(self.arm, poses):
                return 'Failure'

            poses = [self.convertFrameRobotToShelf(self.arm.
                                                   get_current_pose().pose)]            

            poses.append(deepcopy(poses[-1]))
            # To right side of shelf
            poses[-1].position.y = -0.686495  # INCLUDES CURRENT SHELF CALIBRATION as of Saturday night
            poses[-1].position.z += -0.05
            poses[-1].orientation.x = -0.36667
            poses[-1].orientation.y = -0.648119
            poses[-1].orientation.z = 0.333549
            poses[-1].orientation.w = 0.578135

            rospy.loginfo("planning cartesian path to pre-dumping pose")
            if not follow_path(self.arm, poses):
                return 'Failure'

        elif userdata.bin == "J":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path out of bin")
            if not follow_path(self.arm, poses):
                return 'Failure'

            poses = [self.convertFrameRobotToShelf(self.arm.
                                                   get_current_pose().pose)]

            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path to final bin pose")
            if not follow_path(self.arm, poses):
                return 'Failure'

            poses = [self.convertFrameRobotToShelf(self.arm.
                                                   get_current_pose().pose)]            

            poses.append(deepcopy(poses[-1]))
            # To right side of shelf
            poses[-1].position.y = -0.686495  # INCLUDES CURRENT SHELF CALIBRATION as of Saturday night
            poses[-1].position.z += -0.05
            poses[-1].orientation.x = -0.36667
            poses[-1].orientation.y = -0.648119
            poses[-1].orientation.z = 0.333549
            poses[-1].orientation.w = 0.578135

            rospy.loginfo("planning cartesian path to pre-dumping pose")
            if not follow_path(self.arm, poses):
                return 'Failure'

        elif userdata.bin == "K":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path out of bin")
            if not follow_path(self.arm, poses):
                return 'Failure'

            poses = [self.convertFrameRobotToShelf(self.arm.
                                                   get_current_pose().pose)]

            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path to final bin pose")
            if not follow_path(self.arm, poses):
                return 'Failure'

            poses = [self.convertFrameRobotToShelf(self.arm.
                                                   get_current_pose().pose)]            

            poses.append(deepcopy(poses[-1]))
            # To right side of shelf
            poses[-1].position.y = -0.686495  # INCLUDES CURRENT SHELF CALIBRATION as of Saturday night
            poses[-1].position.z += -0.05
            poses[-1].orientation.x = -0.36667
            poses[-1].orientation.y = -0.648119
            poses[-1].orientation.z = 0.333549
            poses[-1].orientation.w = 0.578135

            rospy.loginfo("planning cartesian path to pre-dumping pose")
            if not follow_path(self.arm, poses):
                return 'Failure'

        elif userdata.bin == "L":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path out of bin")
            if not follow_path(self.arm, poses):
                return 'Failure'

            poses = [self.convertFrameRobotToShelf(self.arm.
                                                   get_current_pose().pose)]

            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path to final bin pose")
            if not follow_path(self.arm, poses):
                return 'Failure'

            poses = [self.convertFrameRobotToShelf(self.arm.
                                                   get_current_pose().pose)]            

            poses.append(deepcopy(poses[-1]))
            # To right side of shelf
            poses[-1].position.y = -0.686495  # INCLUDES CURRENT SHELF CALIBRATION as of Saturday night
            poses[-1].position.z += -0.05
            poses[-1].orientation.x = -0.36667
            poses[-1].orientation.y = -0.648119
            poses[-1].orientation.z = 0.333549
            poses[-1].orientation.w = 0.578135

            rospy.loginfo("planning cartesian path to pre-dumping pose")
            if not follow_path(self.arm, poses):
                return 'Failure'


        poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]

        # To order bin
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x = 0.472985  # 0.482178
        poses[-1].position.y = -0.351667  # -0.335627
        poses[-1].position.z = 0.753171  # 0.706449
        poses[-1].orientation.x = -0.164656  # -0.198328
        poses[-1].orientation.y = 0.766477  # 0.759802
        poses[-1].orientation.z = -0.591483  # -0.598499
        poses[-1].orientation.w = -0.188543  # -0.158639
        poses[-1] = self.convertFrameRobotToShelf(poses[-1])

        rospy.loginfo("planning cartesian path to dumping pose")
        if not follow_path(self.arm, poses):
            return 'Failure'

        poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]

        # Tilt little by little
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x = 0.472985  # 0.482178
        poses[-1].position.y = -0.351667  # -0.335627
        poses[-1].position.y += 0.02
        poses[-1].position.z = 0.753171  # 0.706449
        poses[-1].orientation.x = 0.143945
        poses[-1].orientation.y = -0.605741
        poses[-1].orientation.z = 0.757694
        poses[-1].orientation.w = 0.195594
        poses[-1] = self.convertFrameRobotToShelf(poses[-1])

        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x = 0.472985  # 0.482178
        poses[-1].position.y = -0.351667  # -0.335627
        poses[-1].position.y += 0.02
        poses[-1].position.z = 0.753171  # 0.706449
        poses[-1].orientation.x = 0.113945
        poses[-1].orientation.y = -0.525741
        poses[-1].orientation.z = 0.827694
        poses[-1].orientation.w = 0.215594
        poses[-1] = self.convertFrameRobotToShelf(poses[-1])

        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x = 0.472985  # 0.482178
        poses[-1].position.y = -0.351667  # -0.335627
        poses[-1].position.y += 0.02
        poses[-1].position.z = 0.753171  # 0.706449
        poses[-1].orientation.x = 0.112873
        poses[-1].orientation.y = -0.520793
        poses[-1].orientation.z = 0.819904
        poses[-1].orientation.w = 0.209268
        poses[-1] = self.convertFrameRobotToShelf(poses[-1])

        print "Planning Cartesian Path Dump....."
        if not follow_path(self.arm, poses):
            return 'Failure'

        # return right arm to home position
        add_shelf()
        jointValues = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        jointValues[0] = 0.0  # Torso
        jointValues[1] = 1.699523295294849
        jointValues[2] = -0.6448955832339801
        jointValues[3] = -0.06852598822491722
        jointValues[4] = -2.3331612363309975
        jointValues[5] = -0.3915515016420941
        jointValues[6] = 0.15148041914194765
        jointValues[7] = 0.4944912570006051

        # this is currently done instead of goto_pose to use joint values
        self.arm.set_planning_time(5)
        self.arm.set_joint_value_target(jointValues)
        plan = self.arm.plan()
        rospy.loginfo("moving right arm to home position")
        if not self.move(plan.joint_trajectory):
            return 'Failure'

        return 'Success'

    # TODO Use calibrated values, not hardcoded
    def convertFrameRobotToShelf(self, pose):
        pose.position.x += -1.40009583376
        pose.position.y += -0.0841733373195
        pose.position.z += 0.045

        # pose.position.x += -1.4026
        # pose.position.y += -0.0797
        # pose.position.z += 0.045   # 0.048???

        return pose

    def scoopBin(self, horizontalPose):
        remove_shelf()

        rospy.loginfo("planning cartesian path into bin")
        poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]

        # START
        poses.append(horizontalPose)

        # IN
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += 0.155

        # DOWN
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.z += -0.0555

        # IN + DOWN
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += 0.184
        poses[-1].position.z += -0.0810

        # IN
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += 0.1323

        # DOWN
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.z += -0.1018

        # ROTATE BACK/LIFT UP
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += 0.0059
        poses[-1].position.y += 0.0
        poses[-1].position.z += -0.0370
        poses[-1].orientation.x = -0.36665
        poses[-1].orientation.y = -0.64811
        poses[-1].orientation.z = 0.33362
        poses[-1].orientation.w = 0.57811
        # TODO: maybe calibrate pose orientation

        # UP
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.z += 0.08

        # AWAY FROM WALL
        poses.append(deepcopy(poses[-1]))
        if self.rightColumn:
            poses[-1].position.y += -0.05
        elif not self.rightColumn:
            poses[-1].position.y += 0.05
            # STILL NEED TO TEST THIS

        if not follow_path(self.arm, poses):
                return 'Failure'

        return True