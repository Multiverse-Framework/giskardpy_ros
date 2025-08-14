#!/usr/bin/env python
import rospy

from giskardpy_ros.configs.behavior_tree_config import StandAloneBTConfig
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.configs.iai_robots.tracy import TracyWorldConfig, TracyCollisionAvoidanceConfig, \
    TracyStandAloneRobotInterfaceConfig
from giskardpy_ros.ros1.interface import ROS1Wrapper
from giskardpy.middleware import set_middleware

if __name__ == '__main__':
    rospy.init_node('giskard')
    set_middleware(ROS1Wrapper())
    giskard = Giskard(world_config=TracyWorldConfig(),
                      collision_avoidance_config=TracyCollisionAvoidanceConfig(),
                      robot_interface_config=TracyStandAloneRobotInterfaceConfig(),
                      behavior_tree_config=StandAloneBTConfig(include_prefix=True))
    giskard.live()
