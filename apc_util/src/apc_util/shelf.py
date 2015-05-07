
import rospy
import os
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
from collision import scene, remove_object

kiva_pod = os.path.join(os.path.dirname(__file__),
                        "../schedules/meshes/pod_lowres.stl")

PADDING = 0.01

class Shelf(object):
    """Add shelf collision object"""
    NONE = 0
    SIMPLE = 1
    FULL = 2
    PADDED = 3

    def __init__(self, quality):
        super(Shelf, self).__init__()
        self.quality = quality

    def __enter__(self):
        print "Entering shelf: ", self.quality
        if self.quality != Shelf.NONE:
            add_shelf(self.quality)

    def __exit__(self, type, value, tb):
        print "Exit shelf: ", self.quality
        if self.quality != Shelf.NONE:
            remove_shelf()

NO_SHELF = Shelf(Shelf.NONE)
SIMPLE_SHELF = Shelf(Shelf.SIMPLE)
FULL_SHELF = Shelf(Shelf.FULL)
PADDED_SHELF = Shelf(Shelf.PADDED)


def add_shelf(quality=Shelf.SIMPLE):
    pose = get_shelf_pose()
    print "Adding shelf", scene._pub_co.get_num_connections()
    while scene._pub_co.get_num_connections() == 0:
        rospy.sleep(0.01)
        print "Waiting..."
    if quality == Shelf.SIMPLE:
        pose.pose.position.z += 1.25
        scene.add_box(
            name="shelf",
            pose=pose,
            size=(0.96, 2.5, 0.96)
        )
    elif quality == Shelf.FULL:
        scene.add_mesh(
            name="shelf",
            pose=pose,
            filename=kiva_pod
        )
    elif quality == Shelf.PADDED:
        for dx, dy in [(0, 0), (1, 1), (1, -1), (-1, -1), (-1, 1)]:
            mypose = deepcopy(pose)
            mypose.pose.position.x += dx * PADDING
            mypose.pose.position.y += dy * PADDING
            scene.add_mesh(
                name="shelf",
                pose=mypose,
                filename=kiva_pod
            )
    else:
        rospy.logwarn("Unsupported quality %s" % quality)
    print "Added"


def remove_shelf():
    remove_object("shelf")


def get_shelf_pose(prefix="/shelf"):
    pose = PoseStamped()
    pose.header.frame_id = "/base_link"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = rospy.get_param(prefix+"/position/x")
    pose.pose.position.y = rospy.get_param(prefix+"/position/y")
    pose.pose.position.z = rospy.get_param(prefix+"/position/z")
    pose.pose.orientation.x = rospy.get_param(prefix+"/orientation/x")
    pose.pose.orientation.y = rospy.get_param(prefix+"/orientation/y")
    pose.pose.orientation.z = rospy.get_param(prefix+"/orientation/z")
    pose.pose.orientation.w = rospy.get_param(prefix+"/orientation/w")
    return pose


def bin_pose(bin, bin_x=1.32, bin_y=0, bin_z=-0.01):
    # Setting Configuration:
    # 	A		B		C
    # 	D		E		F
    # 	G		H		I
    # 	J		K		L
    # 		   Base

    # Gripper dimension
    GripperLength = 0.2

    # Bin dimension Unit m
    Bin_depth = 0.430
    Start_Gap = 0.100

    LeftBin_width = 0.240
    MiddleBin_width = 0.300
    RightBin_width = 0.240

    WorkBase_Height = 0.820
    BottomLay_Height = 0.230
    SecndLayer_Height = 0.230
    ThirdLayer_Height = 0.220
    TopLayer_Height = 0.260

    Left_horizontal_ShiftValue = MiddleBin_width/2 + LeftBin_width/2
    Right_horizontal_ShiftValue = MiddleBin_width/2 + RightBin_width/2

    TopLayer_vertical_shiftvalue = WorkBase_Height + BottomLay_Height + SecndLayer_Height + ThirdLayer_Height + TopLayer_Height/2
    ThirdLayer_vertical_shiftvalue = WorkBase_Height + BottomLay_Height + SecndLayer_Height + ThirdLayer_Height/2
    SecndLayer_vertical_shiftvalue = WorkBase_Height + BottomLay_Height + SecndLayer_Height/2
    BottomLayer_vertical_shiftvalue = WorkBase_Height + BottomLay_Height/2

    Entry_X_shiftvalue = bin_x - Bin_depth - Start_Gap - GripperLength - 0.035

    pose = PoseStamped()
    pose.pose.position.x = Entry_X_shiftvalue

    if bin == "A":
        pose.pose.position.y = bin_y + Left_horizontal_ShiftValue
        pose.pose.position.z = bin_z + TopLayer_vertical_shiftvalue
    elif bin == "B":
        pose.pose.position.y = bin_y
        pose.pose.position.z = bin_z + TopLayer_vertical_shiftvalue
    elif bin == "C":
        pose.pose.position.y = bin_y - Right_horizontal_ShiftValue
        pose.pose.position.z = bin_z + TopLayer_vertical_shiftvalue
    elif bin == "D":
        pose.pose.position.y = bin_y + Left_horizontal_ShiftValue
        pose.pose.position.z = bin_z + ThirdLayer_vertical_shiftvalue
    elif bin == "E":
        pose.pose.position.y = bin_y
        pose.pose.position.z = bin_z + ThirdLayer_vertical_shiftvalue
    elif bin == "F":
        pose.pose.position.y = bin_y - Right_horizontal_ShiftValue
        pose.pose.position.z = bin_z + ThirdLayer_vertical_shiftvalue
    elif bin == "G":
        pose.pose.position.y = bin_y + Left_horizontal_ShiftValue
        pose.pose.position.z = bin_z + SecndLayer_vertical_shiftvalue
    elif bin == "H":
        pose.pose.position.y = bin_y
        pose.pose.position.z = bin_z + SecndLayer_vertical_shiftvalue
    elif bin == "I":
        pose.pose.position.y = bin_y - Right_horizontal_ShiftValue
        pose.pose.position.z = bin_z + SecndLayer_vertical_shiftvalue
    elif bin == "J":
        pose.pose.position.y = bin_y + Left_horizontal_ShiftValue
        pose.pose.position.z = bin_z + BottomLayer_vertical_shiftvalue
    elif bin == "K":
        pose.pose.position.y = bin_y
        pose.pose.position.z = bin_z + BottomLayer_vertical_shiftvalue
    elif bin == "L":
        pose.pose.position.y = bin_y - Right_horizontal_ShiftValue
        pose.pose.position.z = bin_z + BottomLayer_vertical_shiftvalue
    else:
        raise Exception("Bin `%s` not supported." % bin)
        # TODO: Throw exception

    pose.pose.orientation.x = -0.484592
    pose.pose.orientation.y = 0.384602
    pose.pose.orientation.z = 0.615524
    pose.pose.orientation.w = -0.488244

    return pose
