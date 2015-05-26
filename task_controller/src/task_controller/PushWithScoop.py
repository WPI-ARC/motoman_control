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

        targetBin = userdata.bin
        startBin = "C"

        add_shelf(Shelf.PADDED)
        rospy.loginfo("Trying to push with scoop in bin '"+targetBin+"' ")

        verticalPose = bin_pose_tray(targetBin).pose
        verticalPose.position.x += -0.364436
        verticalPose.position.y += 0.12305766
        verticalPose.position.z += 0.027

        
        # verticalPose.position.x += 0.10
               
        jointConfigVert = [0, 0, 0, 0, 0, 0, 0, 0]

        if targetBin == "A":  # SUCCESS!
            jointConfigVert = [2.5595746841414515, 0.464565161611615,
                               0.7995587587134984, 1.4159480744835042,
                               2.2800272291503196, 1.3764880379224627,
                               1.6300233677200444, -3.047011403282726]
            # jointConfigVert = [2.608074188232422, -0.29658669233322144,
            #                    0.8934586644172668, 1.7289633750915527,
            #                    1.573803424835205, 1.2867212295532227,
            #                    1.4699939489364624, -2.8265552520751953]
            # jointConfigVert = [1.8550660233112988, 1.6181294232479404, -1.9, -1.756166935994761, -1.7730111631047138, 1.957310590193865, 1.9, -0.7786028921501054]
            
            # calibrated pose
            jointConfigVert = [2.6137727006975755, 0.27899243349947583, 0.7389160268552958, 1.4653395180902151, 2.189070343882386, 1.4118932987747352, 1.560457381601488, -3.0869455473464407]

            startBin = "A"
            self.isLeftToRight = True
            rospy.loginfo("Start bin is A")
            # FINE POSITION TUNING FOR INDIVIDUAL BIN
            verticalPose.position.x += 0.000
            verticalPose.position.y += 0.000
            verticalPose.position.z += 0.000

        elif targetBin == "B":  # SUCCESS! ?
            # jointConfigVert = [-2.5986629014403406, -3.13,
            #                    1.124880809692111, -2.1124974525884332,
            #                    2.2554161000398736, -0.7326626517880344,
            #                    -1.6110166411827231, -0.35330049905878025]
            
            # calibrated pose
            jointConfigVert = [2.6137727006975755, 0.27899243349947583,
                               0.7389160268552958, 1.4653395180902151,
                               2.189070343882386, 1.4118932987747352,
                               1.560457381601488, -3.0869455473464407]


            # jointConfigVert = [-2.5978700168593374, -3.13,
            #                    1.1255868983950605, -2.1128080197842634,
            #                    2.25490606375529, -0.7316401551593789,
            #                    -1.6112575301010594, -0.3519577358500504]
            startBin = "B"
            self.isLeftToRight = True
            rospy.loginfo("Start bin is B")
            verticalPose.position.x += 0.000
            verticalPose.position.y += 0.000
            verticalPose.position.z += 0.000

        elif targetBin == "C":  # SUCCESS!??
            # jointConfigVert = [-0.0016260933939344995, -0.18668142488256312,
            #                    -1.1687536891942618, 1.5983031252014734,
            #                    2.290626346507633, 2.6421657959603513,
            #                    -1.1015965088612327, -2.3296530515276337]
            # jointConfigVert = [-0.0016374592439972815, -0.1865226293880318,
            #                    -1.1688353367091011, 1.5983587793581666,
            #                    2.2906470149164506, 2.6426032256816816,
            #                    -1.1016744572816994, -2.32926361562009]
            # jointConfigVert = [0.0040605606931176505, -0.042725640701036766,
            #                        -1.1746684600577848, 1.5461580759549074,
            #                        2.1642181196951555, 2.6403918192892895,
            #                        -1.0958055610071453, -2.3301066646967525]
            
            # calibrated pose?
            jointConfigVert = [0.004158356610633061, -0.043309078200721274,
                               -1.174495592917089, 1.5465294842620085,
                               2.16411681064316, 2.6409477619605117,
                               -1.09564314286045, -2.3294694471020616]


            startBin = "C"
            self.isLeftToRight = False
            rospy.loginfo("Start bin is C")
            verticalPose.position.x += 0.000
            verticalPose.position.y += 0.000
            verticalPose.position.z += 0.000

        elif targetBin == "D":  # SUCCESS!
            # jointConfigVert = [2.905459411335167, 1.9791786685536243,
            #                    0.44221823283736134, -0.15580747083376772,
            #                    2.2349300507549397, 1.8704137525544255,
            #                    1.8999704363459953, -2.7326506516615354]
            # jointConfigVert = [2.70, 2.7412068843841553,
            #                0.09522612392902374, -1.1803146600723267,
            #                2.0825026512145996, 1.8705755472183228,
            #                1.5874949216842651, -2.919917583465576]
            # jointConfigVert = [-2.1951535608033046, 3.018454266779244,
            #                    0.5807550111192726, 0.6298542142851014,
            #                    -2.036710154955131, 1.8853789121877667,
            #                    -0.6420257478504342, 0.4298769819321938]

            # calibrated pose
            jointConfigVert = [-2.2398608492713974, 3.13, 0.5933260431529276, 0.6862511575930882, -1.9933141457383554, 1.813331954923848, -0.7744940568463594, 0.42078248185188505]

            startBin = "D"
            self.isLeftToRight = True
            self.shortRow = True
            rospy.loginfo("Start bin is D")
            verticalPose.position.x += 0.000
            verticalPose.position.y += 0.000
            verticalPose.position.z += 0.000

        elif targetBin == "E":  # mostly success
            # jointConfigVert = [2.89667019844055176, 1.4224945306777954, 
            #                   -0.7801656126976013, -0.2995363175868988,
            #                   2.195582151412964, 1.864424467086792,
            #                   1.6602683067321777, 2.2383474826812744];
            # jointConfigVert = [2.70, 1.4224945306777954, 
            #                   -0.7801656126976013, -0.2995363175868988,
            #                   2.00, 1.864424467086792,
            #                   1.5602683067321777, 2.2383474826812744];

            # calibrated pose
            jointConfigVert = [2.896731181332888, 1.4224328959161416,
                               -0.7801878328930121, -0.29955686652762814,
                               2.19562926767692, 1.8643866096824873,
                               1.6603444430029486, 2.2383027052730324]


            startBin = "E"
            rospy.loginfo("Start bin is E")
            self.shortRow = True
            self.isLeftToRight = True
            verticalPose.position.x += 0.000
            verticalPose.position.y += 0.000
            verticalPose.position.z += 0.000

        elif targetBin == "F":
            # jointConfigVert = [1.5194422006607056,1.810523509979248,
            #                -1.2088792324066162, 1.3328773975372314,
            #                -1.8696491718292236, 1.8829082250595093,
            #                -1.2678426504135132, 1.606799840927124]

            # calibrated pose, THIS CONFIG DOES NOT WORK, has invalid goal state?
            jointConfigVert = [1.7063605500004102,1.1749238170939902,
                               -1.526817689242944, -1.1213039820084452,
                               1.5413271651472102, -1.749239622770513,
                               -1.5439300754469097, 1.7140113060163809];


            startBin = "F"
            rospy.loginfo("Start bin is F")
            self.shortRow = True
            self.isLeftToRight = False
            verticalPose.position.x += 0.000
            verticalPose.position.y += 0.000
            verticalPose.position.z += 0.000

        elif targetBin == "G":
            # jointConfigVert = [2.4718597530192796, 1.1048811085600885,
            #                    1.5289698505492917, -2.1170583249526715,
            #                    -2.089052808535928, -2.178255911290856,
            #                    1.5745535013303766, -1.735037580794114];

            # calibrated pose but torso is flipped around
            jointConfigVert = [-1.72846998032329, 0.527944370647683, -1.285662669980829, 2.2452045118101553, -1.7400052769824363, -1.7215820650867517, -0.6760469606491811, -2.5538767782227345]

            startBin = "G"
            rospy.loginfo("Start bin is G")
            self.shortRow = True
            self.isLeftToRight = True
            verticalPose.position.x += 0.000
            verticalPose.position.y += 0.000
            verticalPose.position.z += 0.000

        elif targetBin == "H":
            # jointConfigVert = [2.95, 1.8770301342010498,
            #                1.2306787967681885, -0.586269199848175,
            #                2.2546935081481934, 1.669684886932373,
            #                1.7160991430282593, 0.7149554491043091]
            # jointConfigVert = [2.5, 1.8770301342010498,
            #                    1.2306787967681885, 0.0,
            #                    1.8046935081481934, 1.669684886932373,
            #                    1.4160991430282593, 0.7149554491043091]

            # calibrated pose??
            jointConfigVert = [2.4718291585873913, 1.1047984538173783, 1.5290256049994881, -2.1169639224415793, -2.0890748066865283, -2.178313072949579, 1.57456751422334, -1.7351008864298179]

            startBin = "H"
            rospy.loginfo("Start bin is H")
            self.shortRow = True
            self.isLeftToRight = True
            verticalPose.position.x += 0.000
            verticalPose.position.y += 0.000
            verticalPose.position.z += 0.000

        elif targetBin == "I":
            # jointConfigVert = [1.5194591283798218, 1.251114845275879,
            #                -1.8047455549240112, 2.224393606185913,
            #                -1.9810069799423218, 1.1204286813735962,
            #                -1.827457070350647, 0.8016403913497925]

            # calibrated pose???
            jointConfigVert = [1.3418513542538393, -1.9163393148721648, 
                               1.8999111796476877, 1.9555683274308242,
                               2.085973354202339, 0.8327696820366999,
                               1.521983626079816, 0.9235781887349414]

            startBin = "I"
            rospy.loginfo("Start bin is I")
            self.shortRow = True
            self.isLeftToRight = False
            verticalPose.position.x += 0.000
            verticalPose.position.y += 0.000
            verticalPose.position.z += 0.000

        elif targetBin == "J":  # PUSH SUCCESS!
            # jointConfigVert = [2.608074188232422, 0.4578932821750641,
            #                1.8810696601867676, -0.5525216460227966,
            #                1.9467278718948364, 0.23977181315422058,
            #                0.7547944784164429, -0.43715447187423706]

            # calibrated pose??
            jointConfigVert = [2.957, 0.5288634230001568, 1.9, -1.1030885861508928, 2.1357994781192318, 0.38148062542826117, 0.4869255952258231, 0.28968305534108246]

            startBin = "J"
            self.isLeftToRight = True
            verticalPose.position.x += 0.000
            verticalPose.position.y += 0.000
            verticalPose.position.z += 0.000
            rospy.loginfo("Start bin is J")

        elif targetBin == "K":
            # jointConfigVert = [2.95, -0.873210072517395,
            #                -0.5380352735519409, 2.7276151180267334,
            #                -2.2068514823913574, 1.085071086883545,
            #                1.8169622421264648, 1.6070705652236938]
            jointConfigVert = [2.70, -0.873210072517395,
                           -0.5380352735519409, 2.7276151180267334,
                           -2.2068514823913574, 1.085071086883545,
                           1.5169622421264648, 1.6070705652236938]
            

            # calibrated pose


            startBin = "K"
            self.isLeftToRight = True
            verticalPose.position.x += 0.000
            verticalPose.position.y += 0.000
            verticalPose.position.z += 0.000
            rospy.loginfo("Start bin is K")

        elif targetBin == "L":
            # jointConfigVert = [1.7551809549331665, 0.04665006324648857,
            #                -1.8453619480133057, 1.8693605661392212,
            #                -1.189427375793457, 1.5698546171188354,
            #                -1.871213436126709, 0.8811066150665283]
            jointConfigVert = [0.9074255864220171,0.08074085792646817,
                               -1.1945154197247605, -1.106070858741708,
                               1.9850889243853769, 1.1827696548230184,
                               1.5891019593508724, -2.824429110575666]

            # calibrated pose


            startBin = "L"
            self.isLeftToRight = False
            rospy.loginfo("Start bin is L")
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
        # if not execute_known_trajectory(self.arm, 'Pick', targetBin):
        # if not execute_known_trajectory(self.arm, 'Pick', startBin):
        #     return 'Failure'
        # remove_shelf()
        # if not execute_known_trajectory(self.arm, 'Dump', targetBin):
        #     return 'Failure'
        # add_shelf()
        # if not execute_known_trajectory(self.arm, 'Lift', targetBin):
        #     return 'Failure'
        # if not execute_known_trajectory(self.arm, 'Home', targetBin):
        #     return 'Failure'
        # return 'Success'
        ##################################################################################

        self.arm.set_planning_time(15)
        self.arm.set_planner_id("RRTConnectkConfigDefault")
        # remove_shelf()  # SHELF SHOULD NOT ACTUALLY BE REMOVED HERE
        add_shelf(Shelf.PADDED)
        self.arm.set_joint_value_target(jointConfigVert)
        plan = self.arm.plan()
        if not self.move(plan.joint_trajectory):
            return 'Failure'

        self.arm.set_pose_reference_frame("/shelf")

        verticalPose = self.convertFrameRobotToShelf(verticalPose)
        rospy.loginfo(verticalPose)
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

        # # THIS BLOCK OF CODE FOR OBTAINING JOINT CONFIGURATIONS FOR PLANNING ONLY, SHOULD BE COMMENTED OUT FOR ACTUAL RUNS
        # remove_shelf()  # SHELF SHOULD NOT ACTUALLY BE REMOVED HERE
        # rospy.sleep(1.0)
        # self.arm.set_pose_target(verticalPose)
        # plan = self.arm.plan()
        # rospy.loginfo("going to vertical pose")
        # if not self.move(plan.joint_trajectory):
        #     rospy.loginfo("FAILED going to vertical pose")
        #     return False

        rospy.loginfo(verticalPose)
        # raw_input("Hit enter to continue ")
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
        xDist = 0.35
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
        xDist = -0.12
        # xDist = -0.02
        poses[-1].position.x += xDist
        if self.isLeftToRight:
            poses[-1].position.y += -0.05
        else:
            poses[-1].position.y += 0.05
        if self.shortRow:
            poses[-1].position.z += xDist*0.0875  # 5 degrees

        poses.append(deepcopy(poses[-1]))
        xDist = -0.35
        poses[-1].position.x += xDist
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
