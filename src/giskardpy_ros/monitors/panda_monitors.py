from __future__ import division
import numpy as np
from typing import Optional, List, Dict, Tuple

import rospy
from geometry_msgs.msg import PointStamped, Point, Vector3
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from visualization_msgs.msg import MarkerArray, Marker

from giskardpy.data_types.data_types import PrefixName, Derivatives, ObservationState
from giskardpy.data_types.exceptions import GoalInitalizationException, ExecutionException
from giskardpy.goals.goal import Goal
from giskardpy.god_map import god_map
from giskardpy.middleware import get_middleware
from giskardpy.model.joints import OmniDrive
from giskardpy.motion_graph.monitors.monitors import Monitor, EndMotion, PayloadMonitor
from giskardpy.symbol_manager import symbol_manager
from giskardpy.motion_graph.tasks.task import WEIGHT_BELOW_CA, WEIGHT_COLLISION_AVOIDANCE
from giskardpy.motion_graph.tasks.pointing import Pointing
import giskardpy.casadi_wrapper as cas
import giskardpy_ros.ros1.msg_converter as msg_converter
from giskardpy.utils.decorators import clear_memo, memoize_with_counter
from giskardpy_ros.tree.blackboard_utils import raise_to_blackboard

class OpenGripper(PayloadMonitor):
    pub = rospy.Publisher('/gripper_command', Float64, queue_size=10)

    def __init__(self, *, value: float=255, name: Optional[str] = None, joint_target: float = 0.04):
        super().__init__(run_call_in_thread=True, name=name)
        self.value = value
        self.ticks = 0
        self.finger_joint = god_map.world.search_for_joint_name('finger_joint1')

    def get_state(self) -> ObservationState:
        return super().get_state()

    def __call__(self):
        if self.state == ObservationState.true:
            return
        msg = Float64()
        msg.data = self.value
        self.pub.publish(msg)
        if self.ticks >= 200:
            self.state = ObservationState.true
        self.ticks += 1

# print(self.name)
        # print(god_map.world.state[self.finger_joint].position)
        # if abs(god_map.world.state[self.finger_joint].position - self.joint_target) <= 0.005:
        #     self.state = ObservationState.true


class CloseGripper(OpenGripper):
    def __init__(self, *, value: float = 0, name: Optional[str] = None):
        super().__init__(value=value, name=name, joint_target=0)