import rospy
import smach

from copy import deepcopy
# from geometry_msgs.msg import Pose, Point, Quaternion
from motoman_moveit.srv import convert_trajectory_server

from apc_util.moveit import follow_path
from apc_util.moveit import goto_pose, execute_known_trajectory, get_known_trajectory
from apc_util.shelf import bin_pose, bin_pose_tray, add_shelf, remove_shelf, Shelf, get_shelf_pose, NO_SHELF, SIMPLE_SHELF, FULL_SHELF, PADDED_SHELF
from apc_util.smach import on_exception



class PushWithScoop(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['bin'], output_keys=[])

        self.arm = robot.arm_right_torso
        # self.armLeft = robot.arm_left_torso
        self.move = rospy.ServiceProxy("/convert_trajectory_service",
                                       convert_trajectory_server)
        self.middleColumn = False
        self.shortRow = False
        self.isLeftToRight = False
        self.startPose = self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)

    @on_exception(failure_state="Failure")
    def execute(self, userdata):
        # return 'Success'

        self.targetBin = userdata.bin
        # startBin = "C"

        add_shelf(Shelf.PADDED)
        rospy.loginfo("Trying to push with scoop in bin '"+self.targetBin+"' ")

        verticalPose = bin_pose_tray(self.targetBin).pose
        verticalPose.position.x += -0.364436
        verticalPose.position.y += 0.12305766
        verticalPose.position.z += 0.027
        # verticalPose.position.z += 0.022

        
        # verticalPose.position.x += 0.10
               
        jointConfigVert = [0, 0, 0, 0, 0, 0, 0, 0]

        if self.targetBin == "A":  # SUCCESS!
            # calibrated pose
            jointConfigVert = [1.9681722954889078, -0.023200618121380513, 1.5089116451880789, 1.8039264439517484, 1.9849145300443676, 1.248039336029401, 1.620441408283454, -3.13]

            self.isLeftToRight = True
            verticalPose.position.x += 0.000
            verticalPose.position.y += 0.000
            verticalPose.position.z += 0.000
            # ^^ FINE POSITION TUNING FOR INDIVIDUAL BINS

        elif self.targetBin == "B":  # SUCCESS!
            # calibrated pose
            jointConfigVert = [-2.691808686026977, -2.8018998306066116, 1.3848981009275314, -2.282453315654881, 1.8152513141302793, -0.6202050989860174, -1.624154936000525, -0.3587748187263247]
 
            self.isLeftToRight = True
            verticalPose.position.x += 0.000
            verticalPose.position.y += 0.000
            verticalPose.position.z += 0.000

        elif self.targetBin == "C":  # SUCCESS!
            # calibrated pose
            jointConfigVert = [0.1128913227811488, 0.17736465719817437, -1.0755894763756846, 1.734991297482921, 1.9132498375426665, 2.425141013887845, -1.0310688499779752, -2.4997632535514924]

            self.isLeftToRight = False
            verticalPose.position.x += 0.000
            verticalPose.position.y += 0.000
            verticalPose.position.z += 0.000

        elif self.targetBin == "D":  # SUCCESS!
            # calibrated pose 
            jointConfigVert = [-2.223934986801738, 3.13, 1.2092354002259527, 0.9307218279859997, -1.8873873503542566, 2.2149979825293564, -1.2486240605659136, 0.28324722321298806]

            self.shortRow = True
            self.isLeftToRight = True
            verticalPose.position.x += 0.000
            verticalPose.position.y += 0.000
            verticalPose.position.z += 0.000

        elif self.targetBin == "E":  # SUCCESS!
            # calibrated pose
            jointConfigVert = [2.9544418528587726, 1.3567262870651748, -1.3266391225690815, -0.22451889273765355, 2.064895928713241, 1.7098359105053893, 1.747522515305617, 2.125112210336924]

            self.shortRow = True
            self.isLeftToRight = True
            verticalPose.position.x += 0.000
            verticalPose.position.y += 0.000
            verticalPose.position.z += 0.000

        elif self.targetBin == "F":  # NO VERTICAL POSE (collides with shelf????)
            # old
            jointConfigVert = [0.1128913227811488, 0.17736465719817437, -1.0755894763756846, 1.734991297482921, 1.9132498375426665, 2.425141013887845, -1.0310688499779752, -2.4997632535514924]

            self.shortRow = True
            self.isLeftToRight = False
            verticalPose.position.x += 0.000
            verticalPose.position.y += 0.000
            verticalPose.position.z += 0.000

        elif self.targetBin == "G":  # SUCCESS!
            # calibrated pose
            jointConfigVert = [-3.0778138189525697, -2.3246408984053084, -0.6736506201571455, -0.946359191105711, -2.4200039133049778, -2.3, -0.31842623885311, 0.9511259625949199]

            self.shortRow = True
            self.isLeftToRight = True
            verticalPose.position.x += 0.000
            verticalPose.position.y += 0.000
            verticalPose.position.z += 0.000

        elif self.targetBin == "H":  # NO VERTICAL POSE - no valid goal states, in collision with cylinder around torso
            # pose FAILED
            jointConfigVert = [2.4718291585873913, 1.1047984538173783, 1.5290256049994881, -2.1169639224415793, -2.0890748066865283, -2.178313072949579, 1.57456751422334, -1.7351008864298179]


            self.shortRow = True
            self.isLeftToRight = True
            verticalPose.position.x += 0.000
            verticalPose.position.y += 0.000
            verticalPose.position.z += 0.000

        elif self.targetBin == "I":  # NO VERTICAL POSE - no valid goal states, in collision with cylinder around torso
            # old
            jointConfigVert = [1.3418513542538393, -1.9163393148721648, 1.8999111796476877, 1.9555683274308242, 2.085973354202339, 0.8327696820366999, 1.521983626079816, 0.9235781887349414]

            self.shortRow = True
            self.isLeftToRight = False
            verticalPose.position.x += 0.000
            verticalPose.position.y += 0.000
            verticalPose.position.z += 0.000

        elif self.targetBin == "J":  # FAILED - pushing items (completed 85%)
            # calibrated pose
            jointConfigVert = [-2.814859477213427, 1.171284271024935, 1.2964470093710962, -1.8496939730019695, 2.154119940035741, -2.417159189716691, 0.29654290371162795, -3.13]

            self.isLeftToRight = True
            verticalPose.position.x += 0.000
            verticalPose.position.y += 0.000
            verticalPose.position.z += 0.000

        elif self.targetBin == "K":  # NO VERTICAL POSE - no valid goal states, in collision with cylinder around torso
            # old
            jointConfigVert = [2.7000695658667015, -0.8731849060533569, -0.5379493727737572, 2.7276986404944212, -2.206815270597627, 1.0851166746411938, 1.5169290144011378, 1.6070088457908016]

            self.isLeftToRight = True
            verticalPose.position.x += 0.000
            verticalPose.position.y += 0.000
            verticalPose.position.z += 0.000

        elif self.targetBin == "L":  # SUCCESS!
            # calibrated pose
            jointConfigVert = [1.2086075801715137, 0.23124532053402494, -1.7309804228879488, -1.2106734273580417, 1.8133929146598422, 1.1998904379674205, 1.7356579754157866, -3.13]

            self.isLeftToRight = False
            verticalPose.position.x += 0.000
            verticalPose.position.y += 0.000
            verticalPose.position.z += 0.000

        # FIX
        leftOffset = 0.000
        middleOffset = 0.000
        rightOffset = 0.000

        # TEMPORARY ##################################################
        verticalPose.position.z += 0.13
        if self.isLeftToRight:
            if self.middleColumn:
                verticalPose.position.y += -0.15
            else:
                verticalPose.position.y += -0.12
        else:
            verticalPose.position.y += 0.12
        ##################################################################

        if self.isLeftToRight:
            if self.shortRow:  # D, E, G, H
                verticalPose.orientation.x = 0.336374
                verticalPose.orientation.y = -0.590811
                verticalPose.orientation.z = -0.250799
                verticalPose.orientation.w = 0.689126
                # TODO: calibrate orientation?
                verticalPose.position.x += -0.050
                verticalPose.position.y += -0.100
                verticalPose.position.z += 0.123
                verticalPose.position.y += leftOffset
                if self.middleColumn:
                    verticalPose.position.y += middleOffset

            else:  # A, B, J, K
                verticalPose.orientation.x = 0.19924
                verticalPose.orientation.y = -0.69387
                verticalPose.orientation.z = -0.14743
                verticalPose.orientation.w = 0.6761
                # TODO: calibrate orientation?
                verticalPose.position.x += 0.000
                verticalPose.position.y += 0.000
                verticalPose.position.z += 0.000
                verticalPose.position.y += leftOffset
                if self.middleColumn:
                    verticalPose.position.y += middleOffset

        elif not self.isLeftToRight:
            if self.shortRow:  # F, I
                verticalPose.orientation.x = -0.659937
                verticalPose.orientation.y = 0.0351767
                verticalPose.orientation.z = 0.738262
                verticalPose.orientation.w = 0.13496
                # TODO: calibrate orientation?
                verticalPose.position.x += -0.050
                verticalPose.position.y += -0.100
                verticalPose.position.z += 0.133
                verticalPose.position.y += rightOffset

            else:  # C, L
                verticalPose.orientation.x = 0.686353
                verticalPose.orientation.y = 0.166894
                verticalPose.orientation.z = -0.680383
                verticalPose.orientation.w = -0.195307
                # TODO: calibrate orientation?
                verticalPose.position.x += -0.018493
                verticalPose.position.y += -.261708
                verticalPose.position.z += 0.025
                verticalPose.position.y += rightOffset

        rospy.loginfo("going to vertical config")

        # TODO: FIX THIS ##################################################################
        # currently calls get_known_trajectory directly to bypass trajectory validation
        # plan, success = get_known_trajectory('Pick', startBin)
        # if not self.move(plan.joint_trajectory):
        #     return 'Failure'

        # plan, success = get_known_trajectory('Dump', startBin)
        # if not self.move(plan.joint_trajectory):
        #     return 'Failure'

        # plan, success = get_known_trajectory('Lift', startBin)
        # if not self.move(plan.joint_trajectory):
        #     return 'Failure'

        # plan, success = get_known_trajectory('Home', startBin)
        # if not self.move(plan.joint_trajectory):
        #     return 'Failure'
        # return 'Success'
        ##################################################################################

        remove_shelf()

        # TODO: USE FOLLOWING CODE BLOCK ONCE PATHS ARE PRE-COMPUTED #####################
        # if not execute_known_trajectory(self.arm, 'Pick', self.targetBin):
        # if not execute_known_trajectory(self.arm, 'Pick', startBin):
        #     return 'Failure'
        # remove_shelf()
        # if not execute_known_trajectory(self.arm, 'Dump', self.targetBin):
        #     return 'Failure'
        # add_shelf()
        # if not execute_known_trajectory(self.arm, 'Lift', self.targetBin):
        #     return 'Failure'
        # if not execute_known_trajectory(self.arm, 'Home', self.targetBin):
        #     return 'Failure'
        # return 'Success'
        ##################################################################################

        self.arm.set_planning_time(15)
        self.arm.set_planner_id("RRTConnectkConfigDefault")
        # remove_shelf()  # SHELF SHOULD NOT ACTUALLY BE REMOVED HERE
        # add_shelf(Shelf.PADDED)
        add_shelf(Shelf.PADDED)
        self.arm.set_joint_value_target(jointConfigVert)
        plan = self.arm.plan()
        if not self.move(plan.joint_trajectory):
            return 'Failure'

        self.arm.set_pose_reference_frame("/shelf")

        verticalPose = self.convertFrameRobotToShelf(verticalPose)
        # rospy.loginfo(verticalPose)
        if not self.pushToSide(verticalPose, jointConfigVert):
            return 'Failure'
        return 'Success'

    def pushToSide(self, verticalPose, jointConfigVert):
        remove_shelf()
        # START
        # rospy.loginfo("going to vertical pose")
        # self.startPose = self.convertFrameRobotToShelf(self.arm.
        #                                                get_current_pose().pose)
        # poses = [self.startPose]

        # poses.append(self.startPose)
        # # poses[-1].position.x = verticalPose.position.x
        # # poses[-1].position.y = verticalPose.position.y
        # # poses[-1].position.z = verticalPose.position.z
        # poses[-1].orientation.x = verticalPose.orientation.x
        # poses[-1].orientation.y = verticalPose.orientation.y
        # poses[-1].orientation.z = verticalPose.orientation.z
        # poses[-1].orientation.w = verticalPose.orientation.w

        # if not follow_path(self.arm, poses):
        #     # rospy.loginfo("FAILED going to vertical position")
        #     rospy.loginfo("FAILED going to vertical orientation")
        #     # rospy.sleep(10)
        #     return False

        # # 
        # poses = [self.convertFrameRobotToShelf(self.arm.
        #                                       get_current_pose().pose)]

        # # poses.append(verticalPose)
        # # poses[-1].position.x += -0.10

        # poses.append(verticalPose)
        # if not follow_path(self.arm, poses):
        #     rospy.loginfo("FAILED going to vertical pose")
        #     # rospy.sleep(10)
        #     return False

        # THIS BLOCK OF CODE FOR OBTAINING JOINT CONFIGURATIONS FOR PLANNING ONLY, SHOULD BE COMMENTED OUT FOR ACTUAL RUNS
        remove_shelf()  # SHELF SHOULD NOT ACTUALLY BE REMOVED HERE
        rospy.sleep(1.0)
        self.arm.set_pose_target(verticalPose)
        plan = self.arm.plan()
        rospy.loginfo("going to vertical pose")
        if not self.move(plan.joint_trajectory):
            rospy.loginfo("FAILED going to vertical pose")
            return False


        verticalPose = self.convertFrameShelfToRobot(verticalPose)
        rospy.loginfo(verticalPose)

        with open("vertical_joint_config.txt", "a+") as out_file:
            joint_config = self.arm.get_current_joint_values()
            out_file.write(str(self.targetBin) + "\t" + str(joint_config) + "\n")
        raw_input("Hit enter to continue ")
        # rospy.sleep(15.0)
        ####################################################################################################

        # IN
        rospy.loginfo("Going along inside wall")
        remove_shelf()
        poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]

        poses.append(deepcopy(poses[-1]))
        xDist = 0.12
        # xDist = 0.02
        poses[-1].position.x += xDist
        if self.isLeftToRight:
            poses[-1].position.y += 0.05
        else:
            poses[-1].position.y += -0.05
        if self.shortRow:
            poses[-1].position.z += -xDist*0.0875  # 5 degrees

        poses.append(deepcopy(poses[-1]))
        # xDist = 0.35
        xDist = 0.25
        poses[-1].position.x += xDist
        if self.isLeftToRight:
            poses[-1].position.y += 0.03
        else:
            poses[-1].position.y += -0.03
        if self.shortRow:
            poses[-1].position.z += -xDist*0.0875  # 5 degrees

        if not follow_path(self.arm, poses):
            rospy.loginfo("FAILED Going along inside wall")
            # rospy.sleep(10)
            return False

        # add_shelf(Shelf.FULL)
        # rospy.sleep(15)
        # remove_shelf()

        # PUSH
        rospy.loginfo("Pushing items to side")
        poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
        # adjust orientation?
        poses.append(deepcopy(poses[-1]))
        if self.isLeftToRight:
            poses[-1].position.y += -0.10
        else:
            poses[-1].position.y += 0.10
        # if self.middleColumn:
        #     if self.isLeftToRight:
        #         poses[-1].position.y += -0.05
        #     else:
        #         poses[-1].position.y += 0.05

        if not follow_path(self.arm, poses):
            rospy.loginfo("FAILED pushing items to side")
            # rospy.sleep(10)
            return False

        # TODO: REPLACE COMPUTE CARTESIAN CALLS WITH ACTUAL REVERSE TRAJECTORIES

        # (reverse) PUSH
        rospy.loginfo("(reverse) Pushing items to side")
        poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
        # adjust orientation?
        poses.append(deepcopy(poses[-1]))
        if self.isLeftToRight:
            poses[-1].position.y += 0.10
        else:
            poses[-1].position.y += -0.10
        # if self.middleColumn:
        #     if self.isLeftToRight:
        #         poses[-1].position.y += -0.05
        #     else:
        #         poses[-1].position.y += 0.05

        if not follow_path(self.arm, poses):
            rospy.loginfo("FAILED (reverse) pushing items to side")
            # rospy.sleep(10)
            return False

        # OUT
        rospy.loginfo("(reverse) Going along inside wall")
        poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]

        poses.append(deepcopy(poses[-1]))
        # xDist = -0.12
        xDist = -0.02
        poses[-1].position.x += xDist
        if self.isLeftToRight:
            poses[-1].position.y += -0.05
        else:
            poses[-1].position.y += 0.05
        if self.shortRow:
            poses[-1].position.z += xDist*0.0875  # 5 degrees

        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += -xDist
        if self.isLeftToRight:
            poses[-1].position.y += -0.03
        else:
            poses[-1].position.y += 0.03
        if self.shortRow:
            poses[-1].position.z += xDist*0.0875  # 5 degrees

        if not follow_path(self.arm, poses):
            rospy.loginfo("FAILED (reverse) Going along inside wall")
            # rospy.sleep(10)
            return False

        # BACK TO START
        # poses = [self.convertFrameRobotToShelf(self.arm.
        #                                        get_current_pose().pose)]
        # rospy.loginfo("Removing tray from bin")
        # # poses.append(deepcopy(poses[-1]))
        # # # poses[-1].position.y += 0.05
        # # # poses[-1].position.x += -0.50 # WORKS FOR BINS A,B,C?
        # # poses[-1].position.x += -0.40
        # # if self.shortRow:
        # #     poses[-1].position.z += xDist*0.0875
        # #     if self.isLeftToRight:
        # #         poses[-1].position.y += 0.05
        # #     else:
        # #         poses[-1].position.y += -0.05

        # poses.append(self.startPose)

        # if not follow_path(self.arm, poses):
        #     return False

        # BACK TO VERTICAL CONFIG
        rospy.loginfo("going back to vertical config")
        add_shelf(Shelf.PADDED)  # MAYBE TODO: GET SIMPLE SHELF TO PASS VALIDITY CHECK AND USE SIMPLE INSTEAD OF PADDED, CURRENTLY TRAY COLLIDES WITH SHELF
        self.arm.set_joint_value_target(jointConfigVert)
        plan = self.arm.plan()
        if not self.move(plan.joint_trajectory):
            rospy.loginfo("FAILED going back to vertical config")
            # rospy.sleep(10)
            return False

        rospy.loginfo("PushWithScoop Success!")
        return True

    def convertFrameRobotToShelf(self, pose):
        shelf_position = get_shelf_pose().pose.position

        pose.position.x += -(shelf_position.x)
        pose.position.y += -(shelf_position.y)
        pose.position.z += -(shelf_position.z)

        # pose.position.x += -1.3535731570096812
        # pose.position.y += -0.08215183129781853
        # pose.position.z += 0.135

        # pose.position.x += -1.39485775456
        # pose.position.y += -0.0744959997413
        # pose.position.z += 0.045

        # pose.position.x += -1.40009583376
        # pose.position.y += -0.0841733373195
        # pose.position.z += 0.045

        return pose

    def convertFrameShelfToRobot(self, pose):
        shelf_position = get_shelf_pose().pose.position

        pose.position.x += (shelf_position.x)
        pose.position.y += (shelf_position.y)
        pose.position.z += (shelf_position.z)

        # pose.position.x += -1.3535731570096812
        # pose.position.y += -0.08215183129781853
        # pose.position.z += 0.135

        # pose.position.x += -1.39485775456
        # pose.position.y += -0.0744959997413
        # pose.position.z += 0.045

        # pose.position.x += -1.40009583376
        # pose.position.y += -0.0841733373195
        # pose.position.z += 0.045

        return pose

# def follow_path(group, path, collision_checking=True):
#     """Follows a cartesian path using a linear interpolation of the given
#     `path`. The `collision_checking` parameter controls whether or not
#     to check the path for collisions with the environment."""
#     traj, success = group.compute_cartesian_path(
#         path,
#         0.01,  # 1cm interpolation resolution
#         0.0,  # jump_threshold
#         avoid_collisions=collision_checking,
#     )

#     if success < 1:
#         rospy.logerr(
#             "Cartesian trajectory could not be completed. Only solved for: '"
#             + str(success) + "'..."
#         )
#         return False

#     if self.move(group, traj.joint_trajectory):
#         return True
#     else:
#         rospy.logerr("Failed to execute cartesian path")
#         return False
