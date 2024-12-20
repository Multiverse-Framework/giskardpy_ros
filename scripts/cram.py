import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, PointStamped

from giskardpy.goals.tracebot import InsertCylinder
from giskardpy.utils.math import quaternion_from_rotation_matrix
from giskardpy_ros.monitors.panda_monitors import OpenGripper, CloseGripper
from giskardpy_ros.python_interface.python_interface import GiskardWrapper
from src.giskardpy_ros.test.utils_for_tests import donbot_urdf


def rotation_matrix(angle_deg):
    # Convert angle from degrees to radians
    angle_rad = np.radians(angle_deg)

    # Define the unit vectors for the base axes
    rotation_y = np.array([1., 0, 0])

    # Compute the z-axis direction based on the angle
    rotation_z = np.array([0., -np.cos(angle_rad), -np.sin(angle_rad)])
    rotation_z /= np.linalg.norm(rotation_z)

    # Compute the x-axis as the cross product of y and z
    rotation_x = np.cross(rotation_y, rotation_z)
    rotation_x /= np.linalg.norm(rotation_x)

    # Construct the rotation matrix
    R = np.vstack([rotation_x, rotation_y, rotation_z, [0., 0, 0]]).T
    R = np.vstack([R, [0., 0, 0, 1]])

    return R

def open_door(start_condition: str) -> str:


    grasp_knobsy_pose = PoseStamped()
    grasp_knobsy_pose.header.frame_id = knobsy_name
    grasp_knobsy_pose.pose.position.z = 0.05

    R = rotation_matrix(90)


    grasp_knobsy_pose.pose.orientation = Quaternion(*quaternion_from_rotation_matrix(R))
    knobsy_pre_grasped = giskard.tasks.add_cartesian_pose(name='grasp_pre_knobsy', root_link=map, tip_link=tool_frame,
                                                      goal_pose=grasp_knobsy_pose,
                                                      start_condition=open)


    grasp_knobsy_pose = PoseStamped()
    grasp_knobsy_pose.header.frame_id = knobsy_name
    grasp_knobsy_pose.pose.position.z = 0.01
    grasp_knobsy_pose.pose.orientation = Quaternion(*quaternion_from_rotation_matrix(R))
    knobsy_grasped = giskard.tasks.add_cartesian_pose(name='grasp_knobsy', root_link=map, tip_link=tool_frame,
                                                      goal_pose=grasp_knobsy_pose,
                                                      start_condition=knobsy_pre_grasped)

    sleep1 = giskard.monitors.add_sleep(name='sleep1', seconds=1, start_condition=knobsy_grasped)

    close = giskard.monitors.add_monitor(class_name=CloseGripper.__name__, name='close_gripper',
                                         start_condition=sleep1)
    sleep2 = giskard.monitors.add_sleep(name='sleep2', seconds=1, start_condition=close)
    return giskard.motion_goals.add_open_container('open',
                                                     tip_link=tool_frame,
                                                     environment_link=knobsy_name,
                                                     goal_joint_state=np.pi/2,
                                                     start_condition=sleep2)

def banananananan(start_condition: str) -> str:
    banana_pose = PoseStamped()
    banana_pose.header.frame_id = 'task_board_banana_plug_red_hole'
    banana_pose.pose.position.z = 0.05
    banana_pose.pose.orientation = Quaternion(*quaternion_from_rotation_matrix([[0, 1, 0, 0],
                                                                                [1, 0, 0, 0],
                                                                                [0, 0, -1, 0],
                                                                                [0,0,0,1.]]))
    pre_banana = giskard.tasks.add_cartesian_pose(name='pre_banana', root_link=map, tip_link=tool_frame,
                                                      goal_pose=banana_pose,
                                                      start_condition=start_condition)
    half_open = giskard.monitors.add_monitor(class_name=OpenGripper.__name__, name='half_open',
                                        value=100,
                                        start_condition=start_condition)
    banana_pose = PoseStamped()
    banana_pose.header.frame_id = 'task_board_banana_plug_red_hole'
    banana_pose.pose.position.z = 0.01
    banana_pose.pose.orientation = Quaternion(*quaternion_from_rotation_matrix([[0, 1, 0, 0],
                                                                                [1, 0, 0, 0],
                                                                                [0, 0, -1, 0],
                                                                                [0,0,0,1.]]))
    banana_grasped = giskard.tasks.add_cartesian_pose(name='grasp_banana', root_link=map, tip_link=tool_frame,
                                                      goal_pose=banana_pose,
                                                      reference_linear_velocity=0.05,
                                                      reference_angular_velocity=0.1,
                                                      start_condition=f'{pre_banana} and {half_open}')
    local_min = giskard.monitors.add_local_minimum_reached(name='wait_banana', start_condition=banana_grasped)
    closed = giskard.monitors.add_monitor(class_name=CloseGripper.__name__, name='closed_bana',
                                        start_condition=local_min)
    banana_up_pose = PoseStamped()
    banana_up_pose.header.frame_id = 'tool_frame'
    banana_up_pose.pose.position.z = -0.25
    banana_up_pose.pose.orientation.w = 1.0
    banana_uped = giskard.tasks.add_cartesian_pose(name='banana up', root_link=map, tip_link=tool_frame,
                                                      goal_pose=banana_up_pose,
                                                      start_condition=closed)
    hole_point = PointStamped()
    hole_point.header.frame_id = 'task_board_banana_plug_black_hole'
    hole_point.point.z = 0.1
    inserted = giskard.motion_goals.add_motion_goal(class_name=InsertCylinder.__name__,
                                                        name='Insert Cyclinder',
                                                        cylinder_name=tool_frame,
                                                        cylinder_height=0.04,
                                                        hole_point=hole_point,
                                                    pre_grasp_height=0.05,
                                                    start_condition=banana_uped)
    push_banana_pose = PoseStamped()
    push_banana_pose.header.frame_id = 'task_board_banana_plug_black_hole'
    push_banana_pose.pose.position.z = 0.01
    push_banana_pose.pose.orientation = Quaternion(*quaternion_from_rotation_matrix([[0, 1, 0, 0],
                                                                                [1, 0, 0, 0],
                                                                                [0, 0, -1, 0],
                                                                                [0,0,0,1.]]))
    push_banana = giskard.tasks.add_cartesian_pose(name='banana push', root_link=map, tip_link=tool_frame,
                                                      goal_pose=push_banana_pose,
                                                   reference_linear_velocity=0.05,
                                                   reference_angular_velocity=0.1,
                                                      start_condition=inserted)
    return push_banana



rospy.init_node('cram')
giskard = GiskardWrapper()
tool_frame = 'tool_frame'
knobsy_name = 'task_board_lid_knob'
map = 'map'

start_pose = {
    'joint1': 0,
    'joint2': 0.45,
    'joint3': -0.02,
    'joint4': -1.35,
    'joint5': 0,
    'joint6': 1.7,
    'joint7': -1.185,

}
joint_goal_reached = giskard.tasks.add_joint_position(name='initial joint pose',
                                 goal_state=start_pose)
open = giskard.monitors.add_monitor(class_name=OpenGripper.__name__, name='open_gripper',
                                    start_condition=joint_goal_reached)
# opened = open_door(open)
done = banananananan(open)


giskard.motion_goals.allow_all_collisions()
giskard.monitors.add_end_motion(done)
giskard.execute()