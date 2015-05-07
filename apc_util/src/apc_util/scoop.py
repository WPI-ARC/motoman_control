
import rospy

import moveit_commander

from moveit import scene

from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene

pubPlanningScene = rospy.Publisher('planning_scene', PlanningScene)
# rospy.wait_for_service('/get_planning_scene', 10.0)
get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)

SCOOP = 'Scoop'

def allow_scoop_collision():
    request = PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
    response = get_planning_scene(request) 
    acm = response.scene.allowed_collision_matrix
    print acm
    if not SCOOP in acm.default_entry_names:
        # add button to allowed collision matrix
        acm.default_entry_names += [SCOOP]
        acm.default_entry_values += [True]
    else:
        acm.default_entry_values[acm.default_entry_names.index(SCOOP)] = True
    print acm.default_entry_names, acm.default_entry_values[acm.default_entry_names.index(SCOOP)], SCOOP in acm.entry_names
    planning_scene_diff = PlanningScene(
            is_diff=True,
            allowed_collision_matrix=acm)
    pubPlanningScene.publish(planning_scene_diff)
    rospy.sleep(1.0)

def disallow_scoop_collision():
    request = PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
    response = get_planning_scene(request) 
    acm = response.scene.allowed_collision_matrix
    print acm
    if not SCOOP in acm.default_entry_names:
        # add button to allowed collision matrix
        acm.default_entry_names += [SCOOP]
        acm.default_entry_values += [False]
    else:
        acm.default_entry_values[acm.default_entry_names.index(SCOOP)] = False
    print acm.default_entry_names, acm.default_entry_values[acm.default_entry_names.index(SCOOP)], SCOOP in acm.entry_names
    planning_scene_diff = PlanningScene(
            is_diff=True,
            allowed_collision_matrix=acm)
    pubPlanningScene.publish(planning_scene_diff)
    rospy.sleep(1.0)

def attach_scoop(group, hand="left"):
    disallow_scoop_collision()
    group.attach_object(
        SCOOP,
        link_name='hand_'+hand+'_palm',
        touch_links=['cylinder_hand_'+hand+'', 'hand_'+hand+'_finger_1_link_0', 
            'hand_'+hand+'_finger_1_link_1', 'hand_'+hand+'_finger_1_link_2', 
            'hand_'+hand+'_finger_1_link_3', 'hand_'+hand+'_finger_2_link_0', 
            'hand_'+hand+'_finger_2_link_1', 'hand_'+hand+'_finger_2_link_2', 
            'hand_'+hand+'_finger_2_link_3', 'hand_'+hand+'_finger_middle_link_0', 
            'hand_'+hand+'_finger_middle_link_1', 'hand_'+hand+'_finger_middle_link_2', 
            'hand_'+hand+'_finger_middle_link_3', 'hand_'+hand+'_palm'],
    )
