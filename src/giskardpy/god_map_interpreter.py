from __future__ import annotations

import rospy

from giskard_msgs.msg import MoveGoal, MoveResult
from giskardpy import identifier
from typing import TYPE_CHECKING, List, Dict, Set, Tuple

from giskardpy.god_map import _GodMap

if TYPE_CHECKING:
    from giskardpy.qp.next_command import NextCommands
    from giskardpy.model.trajectory import Trajectory
    from giskardpy.qp.qp_controller import QPProblemBuilder
    from giskardpy.configs.qp_controller_config import QPControllerConfig
    from giskardpy.my_types import Derivatives, PrefixName
    from giskardpy.goals.goal import Goal
    from giskardpy.configs.giskard import Giskard
    from giskardpy.goals.motion_goal_manager import MotionGoalManager
    from giskardpy.debug_expression_manager import DebugExpressionManager
    from giskardpy.goals.monitors.monitor_manager import MonitorManager
    from giskardpy.goals.monitors.monitors import Monitor
    from giskardpy.configs.collision_avoidance_config import CollisionAvoidanceConfig
    from giskardpy.configs.world_config import WorldConfig
    from giskardpy.tree.control_modes import ControlModes
    from giskardpy.model.collision_world_syncer import CollisionWorldSynchronizer, CollisionCheckerLib, \
    CollisionAvoidanceGroupThresholds, Collisions
    from giskardpy.tree.garden import TreeManager
    from giskardpy.model.world import WorldTree


class GodMap(_GodMap):

    @property
    def goal_id(self) -> int:
        if self.has_data(identifier.goal_id):
            return self.get_data(identifier.goal_id)
        else:
            return -1

    @property
    def time(self) -> float:
        return god_map.get_data(identifier.time)

    @property
    def world(self) -> WorldTree:
        return self.get_data(identifier.world)

    @property
    def monitor_manager(self) -> MonitorManager:
        return self.get_data(identifier.monitor_manager)

    @property
    def motion_goal_manager(self) -> MotionGoalManager:
        return self.get_data(identifier.motion_goal_manager)

    @property
    def giskard(self) -> Giskard:
        return self.get_data(identifier.giskard)

    @property
    def debug_expression_manager(self) -> DebugExpressionManager:
        return self.get_data(identifier.debug_expression_manager)

    @property
    def tree_manager(self) -> TreeManager:
        return self.get_data(identifier.tree_manager)

    @property
    def collision_scene(self) -> CollisionWorldSynchronizer:
        return self.get_data(identifier.collision_scene)

    @property
    def prediction_horizon(self) -> int:
        return self.get_data(identifier.prediction_horizon)

    @property
    def max_derivative(self) -> Derivatives:
        return self.get_data(identifier.max_derivative)

    @property
    def qp_controller(self) -> QPProblemBuilder:
        return self.get_data(identifier.qp_controller)

    @property
    def qp_controller_config(self) -> QPControllerConfig:
        return self.get_data(identifier.qp_controller_config)

    @property
    def collision_checker_id(self) -> CollisionCheckerLib:
        return self.get_data(identifier.collision_checker)

    @property
    def world_config(self) -> WorldConfig:
        return self.get_data(identifier.world_config)

    @property
    def collision_avoidance_config(self) -> CollisionAvoidanceConfig:
        return self.get_data(identifier.collision_avoidance_config)

    @property
    def collision_avoidance_configs(self) -> Dict[str, CollisionAvoidanceGroupThresholds]:
        return self.unsafe_get_data(identifier.collision_avoidance_configs)

    @property
    def trajectory_time_in_seconds(self):
        time = self.get_data(identifier.time)
        if self.is_closed_loop():
            return time
        return time * self.sample_period

    @property
    def sample_period(self) -> float:
        return self.get_data(identifier.sample_period)

    @property
    def goal_msg(self) -> MoveGoal:
        return self.get_data(identifier.goal_msg)

    @property
    def trajectory(self) -> Trajectory:
        return self.get_data(identifier.trajectory)

    @property
    def qp_solver_solution(self) -> NextCommands:
        return self.get_data(identifier.qp_solver_solution)

    @property
    def tmp_folder(self) -> str:
        return self.get_data(identifier.tmp_folder)

    @property
    def goal_package_paths(self) -> Set[str]:
        return self.get_data(identifier.goal_package_paths)

    @property
    def added_collision_checks(self) -> Dict[Tuple[PrefixName, PrefixName], float]:
        return self.get_data(identifier.added_collision_checks)

    @property
    def closest_point(self) -> Collisions:
        return self.get_data(identifier.closest_point)

    @property
    def collision_matrix(self) -> Dict[Tuple[PrefixName, PrefixName], float]:
        return self.get_data(identifier.collision_matrix)

    @property
    def time_delay(self) -> rospy.Duration:
        return self.get_data(identifier.time_delay)

    @property
    def tracking_start_time(self) -> rospy.Time:
        return self.get_data(identifier.tracking_start_time)

    @property
    def result_message(self) -> MoveResult:
        return self.get_data(identifier.result_message)

    def is_goal_msg_type_execute(self):
        return MoveGoal.EXECUTE == self.goal_msg.type

    def is_goal_msg_type_projection(self):
        return MoveGoal.PROJECTION == self.goal_msg.type

    def is_goal_msg_local_minimum_is_success(self):
        return self.goal_msg.local_minimum_is_success

    def is_goal_msg_type_undefined(self):
        return MoveGoal.UNDEFINED == self.goal_msg.type

    def is_closed_loop(self):
        return self.tree_manager.control_mode == self.tree_manager.control_mode.close_loop

    def is_standalone(self):
        return self.tree_manager.control_mode == self.tree_manager.control_mode.standalone

    def is_open_loop(self):
        return self.tree_manager.control_mode == self.tree_manager.control_mode.open_loop

    def is_collision_checking_enabled(self):
        return self.collision_checker_id != self.collision_checker_id.none


god_map = GodMap()
