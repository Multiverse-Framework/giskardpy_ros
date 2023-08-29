from py_trees import Sequence

from giskardpy.god_map_user import GodMapWorshipper
from giskardpy.tree.behaviors.exception_to_execute import ClearBlackboardException
from giskardpy.tree.behaviors.plot_goal_gantt_chart import PlotGanttChart
from giskardpy.tree.behaviors.plot_trajectory import PlotTrajectory
from giskardpy.tree.behaviors.set_move_result import SetMoveResult


class PostProcessing(Sequence, GodMapWorshipper):
    def __init__(self, name: str = 'post processing'):
        super().__init__(name)
        self.add_child(SetMoveResult('set move result', 'Planning'))
        self.add_child(ClearBlackboardException('clear exception'))
        self.add_child(PlotGanttChart())

    def add_plot_trajectory(self, normalize_position: bool = False, wait: bool = False):
        self.add_child(PlotTrajectory('plot trajectory', wait=wait, normalize_position=normalize_position))
