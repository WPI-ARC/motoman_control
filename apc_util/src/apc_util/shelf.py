
import rospy
import os
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
from collision import scene, remove_object
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from services import get_planning_scene

kiva_pod = os.path.join(os.path.dirname(__file__),
                        "../../meshes/pod_lowres.stl")

APC_BINS = "ABCDEFGHIJKL"

PADDING = 0.01
HEIGHT = 2.5
WIDTH = 0.96
DEPTH = 0.96


class Shelf(object):
    """Add shelf collision object"""
    NONE = 0
    SIMPLE = 1
    FULL = 2
    PADDED = 3
    BIN_A = "A"
    BIN_B = "B"
    BIN_C = "C"
    BIN_D = "D"
    BIN_E = "E"
    BIN_F = "F"
    BIN_G = "G"
    BIN_H = "H"
    BIN_I = "I"
    BIN_J = "J"
    BIN_K = "K"
    BIN_L = "L"

    def __init__(self, quality):
        super(Shelf, self).__init__()
        self.quality = quality

    def __enter__(self):
        rospy.loginfo("Adding shelf: %s" % self.quality)
        if self.quality != Shelf.NONE:
            add_shelf(self.quality)
        else:
            rospy.logwarn("Not adding shelf of quality NONE")

    def __exit__(self, type, value, tb):
        rospy.loginfo("Removing shelf: %s" % self.quality)
        if self.quality != Shelf.NONE:
            remove_shelf()
        else:
            rospy.logwarn("Not removing shelf of quality NONE")

NO_SHELF = Shelf(Shelf.NONE)
SIMPLE_SHELF = Shelf(Shelf.SIMPLE)
FULL_SHELF = Shelf(Shelf.FULL)
PADDED_SHELF = Shelf(Shelf.PADDED)


def BIN(bin):
    return Shelf(bin)


def add_shelf(quality=Shelf.SIMPLE):
    pose = get_shelf_pose()
    while scene._pub_co.get_num_connections() == 0:
        rospy.sleep(0.01)
    if quality == Shelf.SIMPLE:
        pose.pose.position.z += HEIGHT/2
        scene.add_box(
            name="shelf",
            pose=pose,
            size=(DEPTH, HEIGHT, WIDTH)
        )
    elif quality == Shelf.FULL:
        scene.add_mesh(
            name="shelf",
            pose=pose,
            filename=kiva_pod
        )
    elif quality == Shelf.PADDED:
        for dx, dy in [(1, 1), (1, -1), (-1, -1), (-1, 1), (0, 0)]:
            mypose = deepcopy(pose)
            mypose.pose.position.x += dx * PADDING
            mypose.pose.position.y += dy * PADDING
            if (dx, dy) == (0, 0):
                name = "shelf"
            else:
                name = "shelf"+str(dx)+str(dy)
            scene.add_mesh(
                name=name,
                pose=mypose,
                filename=kiva_pod
            )
    elif quality in APC_BINS:
        add_bin(quality)
    else:
        rospy.logwarn("Unsupported quality %s" % quality)

    while True:
        rospy.sleep(0.1)
        result, success = get_planning_scene(
            PlanningSceneComponents(
                PlanningSceneComponents.WORLD_OBJECT_NAMES
                + PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            )
        )
        if not success:
            continue
        for object in result.scene.world.collision_objects:
            if object.id == "shelf":
                return


def add_padded_lab():
    rospy.loginfo("Adding padded order bin")
    poses = []
    objects = []

    o_b_pose = Pose()
    o_b_pose.orientation=Quaternion(x=0, y=0, z=0, w=1)
    o_b_pose.position=Point(x=0.5, y=0.2, z=0.22)

    objects.append(SolidPrimitive(
        type=SolidPrimitive.BOX,
        dimensions=[0.43, 0.68, 0.46])
    )
    poses.append(o_b_pose)

    wiring_pose = Pose()
    wiring_pose.orientation=Quaternion(x=0, y=0, z=0, w=1)
    wiring_pose.position=Point(x=-.55, y=0, z=0.15)

    objects.append(SolidPrimitive(
        type=SolidPrimitive.BOX,
        dimensions=[0.6, 0.7, 0.4]
    ))

    poses.append(wiring_pose)

    co = CollisionObject()
    co.operation = CollisionObject.ADD
    co.id = "padded_lab"
    co.header.frame_id = "/base_link"
    co.header.stamp = rospy.Time.now()
    co.primitives = objects
    co.primitive_poses = poses

    scene._pub_co.publish(co)


def remove_padded_lab():
    rospy.loginfo("Removing padded lab")
    remove_object("padded_lab")
    for dx, dy in [(0, 0), (1, 1), (1, -1), (-1, -1), (-1, 1)]:
        remove_object("padded_lab"+str(dx)+str(dy))
    rospy.sleep(5)


def add_bin(bin, prefix="/shelf"):
    # Necessary parameters
    _, max_x, min_y, max_y, min_z, max_z = rospy.get_param(prefix+"/bins/"+bin)
    shelf_max_x = DEPTH/2
    shelf_min_y, shelf_max_y = -WIDTH/2, WIDTH/2
    shelf_min_z, shelf_max_z = 0, HEIGHT
    objects = []
    poses = []

    # Create left side
    objects.append(SolidPrimitive(
        type=SolidPrimitive.BOX,
        dimensions=[DEPTH, abs(shelf_min_y-min_y), HEIGHT],
    ))
    poses.append(Pose(
        position=Point(x=0, y=(shelf_min_y+min_y)/2, z=HEIGHT/2),
        orientation=Quaternion(x=0, y=0, z=0, w=1),
    ))

    # Create right side
    objects.append(SolidPrimitive(
        type=SolidPrimitive.BOX,
        dimensions=[DEPTH, abs(shelf_max_y-max_y), HEIGHT],
    ))
    poses.append(Pose(
        position=Point(x=0, y=(shelf_max_y+max_y)/2, z=HEIGHT/2),
        orientation=Quaternion(x=0, y=0, z=0, w=1),
    ))

    # Create top
    objects.append(SolidPrimitive(
        type=SolidPrimitive.BOX,
        dimensions=[DEPTH, abs(max_y-min_y), abs(shelf_max_z-max_z)],
    ))
    poses.append(Pose(
        position=Point(x=0, y=(max_y+min_y)/2, z=(shelf_max_z+max_z)/2),
        orientation=Quaternion(x=0, y=0, z=0, w=1),
    ))

    # Create bottom
    objects.append(SolidPrimitive(
        type=SolidPrimitive.BOX,
        dimensions=[DEPTH, abs(max_y-min_y), abs(shelf_min_z-min_z)],
    ))
    poses.append(Pose(
        position=Point(x=0, y=(max_y+min_y)/2, z=(shelf_min_z+min_z)/2),
        orientation=Quaternion(x=0, y=0, z=0, w=1),
    ))

    # Create back
    objects.append(SolidPrimitive(
        type=SolidPrimitive.BOX,
        dimensions=[DEPTH/2, abs(max_y-min_y), abs(max_z-min_z)],
    ))
    poses.append(Pose(
        position=Point(x=shelf_max_x/2, y=(max_y+min_y)/2, z=(min_z+max_z)/2),
        orientation=Quaternion(x=0, y=0, z=0, w=1),
    ))

    co = CollisionObject()
    co.operation = CollisionObject.ADD
    co.id = "shelf"
    co.header.frame_id = "/shelf"
    co.header.stamp = rospy.Time.now()
    co.primitives = objects
    co.primitive_poses = poses

    scene._pub_co.publish(co)


def remove_shelf():
    remove_object("shelf")
    for dx, dy in [(0, 0), (1, 1), (1, -1), (-1, -1), (-1, 1)]:
        remove_object("shelf"+str(dx)+str(dy))


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
    shelf = get_shelf_pose().pose

    bin_x, bin_y, bin_z = shelf.position.x, shelf.position.y, shelf.position.z

    # Gripper dimension
    GripperLength = 0.2

    # Bin dimension Unit m
    Bin_depth = 0.430
    Start_Gap = 0.100

    LeftBin_width = 0.240
    MiddleBin_width = 0.300
    RightBin_width = 0.240

    # WorkBase_Height = 0.790
    WorkBase_Height = 0.820 - 0.090
    BottomLay_Height = 0.270
    SecndLayer_Height = 0.230
    ThirdLayer_Height = 0.230
    TopLayer_Height = 0.270

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


def bin_pose_tray(bin, bin_x=1.32, bin_y=0, bin_z=-0.01):
    # Setting Configuration:
    #   A       B       C
    #   D       E       F
    #   G       H       I
    #   J       K       L
    #          Base
    shelf = get_shelf_pose().pose

    bin_x, bin_y, bin_z = shelf.position.x, shelf.position.y, shelf.position.z

    # Gripper dimension
    GripperLength = 0.2

    # Bin dimension Unit m
    Bin_depth = 0.430
    Start_Gap = 0.100

    LeftBin_width = 0.240
    MiddleBin_width = 0.300
    RightBin_width = 0.240

    WorkBase_Height = 0.790
    # WorkBase_Height = 0.820 - 0.090
    # BottomLay_Height = 0.265
    BottomLay_Height = 0.270
    SecndLayer_Height = 0.230
    ThirdLayer_Height = 0.230
    # TopLayer_Height = 0.265
    TopLayer_Height = 0.270

    Left_horizontal_ShiftValue = MiddleBin_width/2 + LeftBin_width
    Right_horizontal_ShiftValue = MiddleBin_width/2 + RightBin_width
    Middle_horiztonal_ShiftValue = MiddleBin_width/2

    TopLayer_vertical_shiftvalue = WorkBase_Height + BottomLay_Height + SecndLayer_Height + ThirdLayer_Height
    ThirdLayer_vertical_shiftvalue = WorkBase_Height + BottomLay_Height + SecndLayer_Height
    SecndLayer_vertical_shiftvalue = WorkBase_Height + BottomLay_Height
    BottomLayer_vertical_shiftvalue = WorkBase_Height

    Entry_X_shiftvalue = bin_x - Bin_depth - Start_Gap - GripperLength - 0.035

    pose = PoseStamped()
    pose.pose.position.x = Entry_X_shiftvalue

    if bin == "A":
        pose.pose.position.y = bin_y + Left_horizontal_ShiftValue
        pose.pose.position.z = bin_z + TopLayer_vertical_shiftvalue
    elif bin == "B":
        pose.pose.position.y = bin_y + Middle_horiztonal_ShiftValue
        pose.pose.position.z = bin_z + TopLayer_vertical_shiftvalue
    elif bin == "C":
        pose.pose.position.y = bin_y - Right_horizontal_ShiftValue
        pose.pose.position.z = bin_z + TopLayer_vertical_shiftvalue
    elif bin == "D":
        pose.pose.position.y = bin_y + Left_horizontal_ShiftValue
        pose.pose.position.z = bin_z + ThirdLayer_vertical_shiftvalue
    elif bin == "E":
        pose.pose.position.y = bin_y + Middle_horiztonal_ShiftValue
        pose.pose.position.z = bin_z + ThirdLayer_vertical_shiftvalue
    elif bin == "F":
        pose.pose.position.y = bin_y - Right_horizontal_ShiftValue
        pose.pose.position.z = bin_z + ThirdLayer_vertical_shiftvalue
    elif bin == "G":
        pose.pose.position.y = bin_y + Left_horizontal_ShiftValue
        pose.pose.position.z = bin_z + SecndLayer_vertical_shiftvalue
    elif bin == "H":
        pose.pose.position.y = bin_y + Middle_horiztonal_ShiftValue
        pose.pose.position.z = bin_z + SecndLayer_vertical_shiftvalue
    elif bin == "I":
        pose.pose.position.y = bin_y - Right_horizontal_ShiftValue
        pose.pose.position.z = bin_z + SecndLayer_vertical_shiftvalue
    elif bin == "J":
        pose.pose.position.y = bin_y + Left_horizontal_ShiftValue
        pose.pose.position.z = bin_z + BottomLayer_vertical_shiftvalue
    elif bin == "K":
        pose.pose.position.y = bin_y + Middle_horiztonal_ShiftValue
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
