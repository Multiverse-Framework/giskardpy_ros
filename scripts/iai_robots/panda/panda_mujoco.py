#!/usr/bin/env python
from typing import Optional

import numpy as np
import rospy

from giskardpy.data_types.data_types import PrefixName
from giskardpy_ros.configs.giskard import Giskard
from giskardpy.model.world_config import WorldWithFixedRobot
from giskardpy.qp.qp_controller_config import QPControllerConfig
from giskardpy.middleware import set_middleware
from giskardpy_ros.ros1.interface import ROS1Wrapper
from giskardpy_ros.configs.giskard import RobotInterfaceConfig
from giskardpy_ros.configs.behavior_tree_config import OpenLoopBTConfig, ClosedLoopBTConfig
from giskardpy.model.collision_avoidance_config import CollisionAvoidanceConfig
from giskardpy.model.collision_world_syncer import CollisionCheckerLib


class PandaJointTrajServerIAIInterface(RobotInterfaceConfig):
    def setup(self):
        self.sync_joint_state_topic('/world/panda/joint_states', group_name='panda')
        self.add_follow_joint_trajectory_server(namespace='/world/panda/joint_trajectory_controller',
                                                fill_velocity_values=True)


class PandaJointVelServerIAIInterface(RobotInterfaceConfig):
    def setup(self):
        self.sync_joint_state_topic('/world/panda/joint_states', group_name='panda')
        self.add_joint_velocity_group_controller(namespace='world/panda/joint_group_velocity_controller')

class PandaCollisionAvoidance(CollisionAvoidanceConfig):
    def __init__(self, collision_checker: CollisionCheckerLib = CollisionCheckerLib.bpb):
        super().__init__(collision_checker=collision_checker)

    def setup(self):
        self.load_self_collision_matrix('self_collision_matrices/panda.srdf')

class PandaWorldConfig(WorldWithFixedRobot):
    def setup(self, robot_name: Optional[str] = None) -> None:
        super().setup(robot_name=robot_name)
        self.add_empty_link(link_name=PrefixName('tool_frame'))
        self.add_fixed_joint(parent_link='hand', child_link='tool_frame',
                             homogenous_transform=np.array([[1,0,0,0],
                                                            [0,1,0,0],
                                                            [0,0,1,0.1],
                                                            [0,0,0,1]]))

        box_urdf = rospy.get_param('task_board_description')
        group_name = self.add_robot_urdf(box_urdf)
        root_link_name = self.get_root_link_of_group(group_name)

        self.add_fixed_joint(parent_link="map", child_link=root_link_name,
                             # homogenous_transform=np.array([[0, -1, 0, 0.6],
                             #                                [1, 0, 0, 0],
                             #                                [0, 0, 1, 0.08],
                             #                                [0, 0, 0, 1]])
                             homogenous_transform=np.array([[1, 0, 0, 0.6],
                                                            [0, 1, 0, 0],
                                                            [0, 0, 1, 0.08],
                                                            [0, 0, 0, 1]])
                             )


if __name__ == '__main__':
    rospy.init_node('giskard')
    set_middleware(ROS1Wrapper())
    drive_joint_name = 'brumbrum'
    urdf = rospy.get_param('robot_description')
    giskard = Giskard(world_config=PandaWorldConfig(urdf=urdf),
                      collision_avoidance_config=PandaCollisionAvoidance(),
                      robot_interface_config=PandaJointVelServerIAIInterface(),
                      behavior_tree_config=ClosedLoopBTConfig(debug_mode=True),
                      qp_controller_config=QPControllerConfig(mpc_dt=0.01,
                                                              prediction_horizon=20,
                                                              control_dt=0.01))
    giskard.live()
