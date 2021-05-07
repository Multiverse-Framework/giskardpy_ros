from collections import OrderedDict
from time import time

import numpy as np

from giskardpy import logging, casadi_wrapper as w
from giskardpy.data_types import JointConstraint
from giskardpy.data_types import SoftConstraint
from giskardpy.exceptions import QPSolverException, InfeasibleException, OutOfJointLimitsException, \
    HardConstraintsViolatedException
from giskardpy.qp_solver import QPSolver
from giskardpy.utils import make_filter_masks, create_path


def print_pd_dfs(dfs, names):
    import pandas as pd
    import datetime
    folder_name = u'debug_matrices/{}'.format(datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
    for df, name in zip(dfs, names):
        path = u'{}/{}.debug'.format(folder_name, name)
        create_path(path)
        with pd.option_context('display.max_rows', None, 'display.max_columns', None):
            with open(path, 'w') as f:
                f.write(df.to_csv())


class Parent(object):
    def _sorter(self, jv=None, ja=None, jj=None, sv=None, sa=None):
        """
        :type jv: dict
        :type ja: dict
        :type sv: dict
        :type sa: dict
        :return: list
        """
        jv = jv if jv is not None else {}
        ja = ja if ja is not None else {}
        jj = jj if jj is not None else {}
        sv = sv if sv is not None else {}
        sa = sa if sa is not None else {}
        return self.__helper(jv) + self.__helper(ja) + self.__helper(jj) + self.__helper(sv) + self.__helper(sa), \
               self.__helper_names(jv) + \
               self.__helper_names(ja) + \
               self.__helper_names(jj) + \
               self.__helper_names(sv) + \
               self.__helper_names(sa)

    def __helper(self, param):
        return [x for _, x in sorted(param.items())]

    def __helper_names(self, param):
        return [x for x, _ in sorted(param.items())]


class H(Parent):
    def __init__(self):
        self.__j_weights_v = {}
        self.__j_weights_a = {}
        self.__j_weights_j = {}
        self.__s_weights_v = {}
        self.__s_weights_a = {}

    def add_joint_constraint(self, name, constraint):
        """
        :type name: str
        :type constraint: JointConstraint
        :return:
        """
        self.__j_weights_v[name + '/v'] = constraint.weight_v
        self.__j_weights_a[name + '/a'] = constraint.weight_a
        self.__j_weights_j[name + '/j'] = constraint.weight_j

    def add_soft_constraint(self, name, constraint):
        """
        :type name: str
        :type constraint: SoftConstraint
        :return:
        """
        self.__s_weights_v[name + '/v'] = constraint.weight_v
        self.__s_weights_a[name + '/a'] = constraint.weight_a

    def weights(self):
        return self._sorter(jv=self.__j_weights_v,
                            ja=self.__j_weights_a,
                            jj=self.__j_weights_j,
                            sv=self.__s_weights_v,
                            sa=self.__s_weights_a)[0]


class B(Parent):
    def __init__(self):
        self._j_lb_v = {}
        self._j_ub_v = {}
        self._j_lb_a = {}
        self._j_ub_a = {}
        self._j_lb_j = {}
        self._j_ub_j = {}
        self._s_lb_v = {}
        self._s_ub_v = {}
        self._s_lb_a = {}
        self._s_ub_a = {}

    def add_joint_constraint(self, name, constraint):
        """
        :type name: str
        :type constraint: JointConstraint
        """
        self._j_lb_v[name + '/v'] = constraint.lower_v
        self._j_lb_a[name + '/a'] = constraint.lower_a
        self._j_lb_j[name + '/j'] = constraint.lower_j
        self._j_ub_v[name + '/v'] = constraint.upper_v
        self._j_ub_a[name + '/a'] = constraint.upper_a
        self._j_ub_j[name + '/j'] = constraint.upper_j

    def add_soft_constraint(self, name, constraint):
        """
        :type name: str
        :type constraint: SoftConstraint
        :return:
        """
        self._s_lb_v[name + '/v'] = constraint.lower_slack_limit_v
        self._s_lb_a[name + '/a'] = constraint.lower_slack_limit_a
        self._s_ub_v[name + '/v'] = constraint.upper_slack_limit_v
        self._s_ub_a[name + '/a'] = constraint.upper_slack_limit_a

    def lb(self):
        return self._sorter(jv=self._j_lb_v,
                            ja=self._j_lb_a,
                            jj=self._j_lb_j,
                            sv=self._s_lb_v,
                            sa=self._s_lb_a)[0]

    def ub(self):
        return self._sorter(jv=self._j_ub_v,
                            ja=self._j_ub_a,
                            jj=self._j_ub_j,
                            sv=self._s_ub_v,
                            sa=self._s_ub_a)[0]

    def names(self):
        return self._sorter(jv=self._j_ub_v,
                            ja=self._j_ub_a,
                            jj=self._j_ub_j,
                            sv=self._s_ub_v,
                            sa=self._s_ub_a)[1]


class BA(Parent):
    def __init__(self):
        self._lbA_v = {}
        self._ubA_v = {}
        self._lbA_a = {}
        self._ubA_a = {}
        self._j_lbA_a_link = {}
        self._j_ubA_a_link = {}
        self._j_lbA_j_link = {}
        self._j_ubA_j_link = {}

    def add_joint_constraint(self, name, constraint):
        """
        :type name: str
        :type constraint: JointConstraint
        """
        self._j_lbA_a_link[name + '/acc_link'] = constraint.joint_velocity_symbol
        self._j_ubA_a_link[name + '/acc_link'] = constraint.joint_velocity_symbol
        self._j_lbA_j_link[name + '/jerk_link'] = constraint.joint_acceleration_symbol
        self._j_ubA_j_link[name + '/jerk_link'] = constraint.joint_acceleration_symbol

    def add_soft_constraint(self, name, constraint):
        """
        :type name: str
        :type constraint: SoftConstraint
        """
        self._lbA_v[name + '/v'] = constraint.lbA_v
        self._lbA_a[name + '/a'] = constraint.lbA_a
        self._ubA_v[name + '/v'] = constraint.ubA_v
        self._ubA_a[name + '/a'] = constraint.ubA_a

    def lbA(self):
        return self._sorter(ja=self._j_lbA_a_link,
                            jj=self._j_lbA_j_link,
                            sv=self._lbA_v,
                            sa=self._lbA_a)[0]

    def ubA(self):
        return self._sorter(ja=self._j_ubA_a_link,
                            jj=self._j_ubA_j_link,
                            sv=self._ubA_v,
                            sa=self._ubA_a)[0]

    def names(self):
        return self._sorter(ja=self._j_ubA_a_link,
                            jj=self._j_ubA_j_link,
                            sv=self._ubA_v,
                            sa=self._ubA_a)[1]


class A(Parent):
    def __init__(self, sample_period):
        self._A_soft = {}
        self._A_hard = {}
        self._A_joint = {}
        self.sample_period = sample_period

    def add_joint_constraint(self, name, constraint):
        """
        :type name: str
        :type constraint: JointConstraint
        :return:
        """
        self._A_joint[name] = constraint.joint_symbol

    def add_hard_constraint(self, name, constraint):
        self._A_hard[name] = constraint.expression

    def add_soft_constraint(self, name, constraint):
        """
        :type name:
        :type constraint: SoftConstraint
        """
        self._A_soft[name] = constraint.expression

    def controlled_joint_symbols(self):
        return self._sorter(jv=self._A_joint)[0]

    def construct_A_soft(self):
        j = len(self._A_joint)
        s = len(self._A_soft)
        A_soft = w.zeros(j * 2 + s * 2, j * 3 + s * 2)
        soft_expressions = self._sorter(sv=self._A_soft)[0]
        controlled_joints = self.controlled_joint_symbols()
        t = time()
        jac = w.jacobian(w.Matrix(soft_expressions), controlled_joints, order=1)
        logging.loginfo(u'computed Jacobian in {:.5f}s'.format(time() - t))
        jac_dot = w.jacobian(w.Matrix(soft_expressions), controlled_joints, order=2)
        logging.loginfo(u'computed Jacobian dot in {:.5f}s'.format(time() - t))

        #        j    j    j    s    s
        # j   |  I | -I |    |    |    |
        # j   |    |  I | -I |    |    |
        # sv  | J  |    |    |  I |    |
        # sa  | Jd | J  |    |    |  I |
        #

        # this assumes a jv ja sorting
        # row 1
        v_start = 0
        v_end = j
        A_soft[v_start:v_end, :j * 1] = w.eye(j)
        A_soft[v_start:v_end, j * 1:j * 2] = -w.eye(j) * self.sample_period

        # row 2
        v_start += j
        v_end += j
        A_soft[v_start:v_end, j * 1:j * 2] = w.eye(j)
        A_soft[v_start:v_end, j * 2:j * 3] = -w.eye(j)

        # row 3
        v_start += j
        v_end +=  s
        A_soft[v_start:v_end, :j * 1] = jac * self.sample_period
        A_soft[v_start:v_end, j * 3:j * 3 + s] = w.eye(s)

        # row 4
        v_start += s
        v_end += s
        A_soft[v_start:v_end, :j * 1] = jac_dot * self.sample_period
        A_soft[v_start:v_end, j * 1:j * 2] = jac
        A_soft[v_start:v_end, j * 3 + s:] = w.eye(s)
        return A_soft

    def A(self):
        return self.construct_A_soft()


class QProblemBuilder(object):
    """
    Wraps around QPOases. Builds the required matrices from constraints.
    """

    def __init__(self, joint_constraints_dict, hard_constraints_dict, soft_constraints_dict, sample_period,
                 path_to_functions=''):
        """
        :type joint_constraints_dict: dict
        :type hard_constraints_dict: dict
        :type soft_constraints_dict: dict
        :type controlled_joint_symbols: list
        :param path_to_functions: location where the compiled functions can be safed.
        :type path_to_functions: str
        """
        self.b = B()
        self.H = H()
        self.bA = BA()
        self.A = A(sample_period)
        self.order = 2
        self.path_to_functions = path_to_functions
        self.joint_constraints_dict = joint_constraints_dict
        self.hard_constraints_dict = hard_constraints_dict
        self.soft_constraints_dict = soft_constraints_dict
        self.construct_big_ass_M()
        self.compile_big_ass_M()

        self.num_hard_constraints = len(self.hard_constraints_dict)
        self.num_joint_constraints = len(self.joint_constraints_dict)
        self.num_soft_constraints = len(self.soft_constraints_dict)

        self.qp_solver = QPSolver()
        self.lbAs = None  # for debugging purposes

    def get_joint_symbols(self):
        joint_symbols = []
        for constraint_name, constraint in self.joint_constraints_dict.items():  # type: (str, JointConstraint)
            joint_symbols.append(constraint.joint_symbol)
        return sorted(joint_symbols)

    def get_expr(self):
        return self.compiled_big_ass_M.str_params

    @profile
    def compile_big_ass_M(self):
        t = time()
        self.free_symbols = w.free_symbols(self.big_ass_M)
        self.compiled_big_ass_M = w.speed_up(self.big_ass_M,
                                             self.free_symbols)
        logging.loginfo(u'compiled symbolic expressions in {:.5f}s'.format(time() - t))

    def are_joint_limits_violated(self, p_lb, p_ub):
        violations = (p_ub - p_lb)[p_lb.data > p_ub.data]
        if len(violations) > 0:
            logging.logerr(u'The following joints are outside of their limits: \n {}'.format(violations))
            return True
        logging.loginfo(u'All joints are within limits')
        return False

    def save_all(self, weights, A, lbA, ubA, lb, ub, xdot=None):
        if xdot is not None:
            print_pd_dfs([weights, A, lbA, ubA, lb, ub, xdot],
                         ['weights', 'A', 'lbA', 'ubA', 'lb', 'ub', 'xdot'])
        else:
            print_pd_dfs([weights, A, lbA, ubA, lb, ub],
                         ['weights', 'A', 'lbA', 'ubA', 'lb', 'ub'])

    def is_nan_in_array(self, name, p_array):
        p_filtered = p_array.apply(lambda x: zip(x.index[x.isnull()].tolist(), x[x.isnull()]), 1)
        p_filtered = p_filtered[p_filtered.apply(lambda x: len(x)) > 0]
        if len(p_filtered) > 0:
            logging.logerr(u'{} has the following nans:'.format(name))
            self.print_pandas_array(p_filtered)
            return True
        logging.loginfo(u'{} has no nans'.format(name))
        return False

    def print_pandas_array(self, array):
        import pandas as pd
        if len(array) > 0:
            with pd.option_context('display.max_rows', None, 'display.max_columns', None):
                print(array)

    def init_big_ass_M(self):
        """
        #          vel  acc  jerk
        #           j    j    j     s    s        1      1      1
        #         |-----------------------------------------------|
        # h       |    custom     |    0      | lbA_h | ubA_h | 0 | hard constraints
        #         |-----------------------------------------------|
        # j       | I  | -I |  0  |  0  |  0  | last v| last v| 0 | last velocity = current velocity - current acceleration
        # j       | 0  |  I | -I  |  0  |  0  | last a| last a| 0 | last acceleration = current acceleration - current jerk
        # s       | J  | 0  |  0  |  I  |  0  | sv lbA| sv ubA| 0 | vel soft constraints
        # s       | Jd | J  |  0  |  0  |  I  | sa lbA| sa ubA| 0 | acc soft constraints
        #         |-----------------------------------------------|
        # j vel   | wv                        | jv lb | jv ub |   |
        # j acc   |      wa                   | ja lb | ja ub | g |  weights
        # j jerk  |           wj              | jj lb | jj ub |   |
        # s       |                 wsv       | sv lb | sv ub |   |
        # s       |                       wsa | sa lb | sa ub |   |
        #         |-----------------------------------------------|
        """
        self.big_ass_M = w.zeros(self.h + self.j * 2 + self.s * 2 + self.j * 3 + self.s * 2,
                                 self.j * 3 + self.s * 2 + 3)
        self.A_vertical_start = self.h
        self.A_vertical_stop = self.A_vertical_start + self.j * 2 + self.s * 2
        self.A_horizontal_start = 0
        self.A_horizontal_stop = self.j * 3 + self.s * 2
        self.H_vertical_start = self.A_vertical_stop
        self.H_vertical_stop = None
        self.H_horizontal_start = 0
        self.H_horizontal_stop = self.A_horizontal_stop
        self.A_height = self.A_vertical_stop - self.A_vertical_start
        self.A_length = self.A_horizontal_stop - self.A_horizontal_start

    def set_A_soft(self, A_soft):
        self.big_ass_M[self.A_vertical_start:self.A_vertical_stop,
        self.A_horizontal_start:self.A_horizontal_stop] = A_soft

    def set_weights(self, weights):
        self.big_ass_M[self.H_vertical_start:self.H_vertical_stop,
        self.H_horizontal_start:self.H_horizontal_stop] = w.diag(*weights)

    def set_lb(self, lb):
        self.big_ass_M[self.H_vertical_start:self.H_vertical_stop, self.H_horizontal_stop] = lb

    def set_ub(self, ub):
        self.big_ass_M[self.H_vertical_start:self.H_vertical_stop, self.H_horizontal_stop + 1] = ub

    def set_linear_weights(self, linear_weights):
        self.big_ass_M[self.H_vertical_start:self.H_vertical_stop, self.H_horizontal_stop + 2] = linear_weights

    def set_lbA(self, lbA):
        self.big_ass_M[self.A_vertical_start:self.A_vertical_stop, self.H_horizontal_stop] = lbA

    def set_ubA(self, ubA):
        self.big_ass_M[self.A_vertical_start:self.A_vertical_stop, self.H_horizontal_stop + 1] = ubA

    def set_A_hard(self, hard_expressions):
        for i, row in enumerate(hard_expressions):
            self.big_ass_M[i, :self.j * 2] = row

    def filter_zero_weight_constraints(self, H, A, lb, ub, lbA, ubA, g):
        bA_mask, b_mask = make_filter_masks(H, self.num_joint_constraints, self.num_hard_constraints)
        A = A[bA_mask][:, b_mask].copy()
        lbA = lbA[bA_mask]
        ubA = ubA[bA_mask]
        lb = lb[b_mask]
        ub = ub[b_mask]
        g = g[b_mask]
        H = H[b_mask][:, b_mask]
        return H, A, lb, ub, lbA, ubA, g

    @profile
    def construct_big_ass_M(self):
        # TODO cpu intensive
        soft_expressions = []
        hard_expressions = []
        for i, ((_, constraint_name), constraint) in enumerate(
                self.joint_constraints_dict.items()):  # type: (str, JointConstraint)
            self.H.add_joint_constraint(constraint_name, constraint)
            self.b.add_joint_constraint(constraint_name, constraint)
            self.bA.add_joint_constraint(constraint_name, constraint)
            self.A.add_joint_constraint(constraint_name, constraint)

        # for constraint_name, constraint in self.hard_constraints_dict.items():  # type: (str, HardConstraint)
        #     self.bA.add(name=constraint_name,
        #                 lower_v=constraint.lower,
        #                 upper_v=constraint.upper,
        #                 lower_a=constraint.)
        #     lbA.append(constraint.lower)
        #     ubA.append(constraint.upper)
        #     hard_expressions.append(constraint.expression)

        for constraint_name, constraint in self.soft_constraints_dict.items():  # type: (str, SoftConstraint)
            self.H.add_soft_constraint(constraint_name, constraint)
            self.b.add_soft_constraint(constraint_name, constraint)
            self.bA.add_soft_constraint(constraint_name, constraint)
            self.A.add_soft_constraint(constraint_name, constraint)

        logging.loginfo(u'constructing new controller with {} soft constraints...'.format(len(self.bA_names())))
        # assert len(self.hard_constraints_dict) == 0, 'hard constraints are not supported anymore'
        self.h = len(self.hard_constraints_dict)
        self.s = len(self.soft_constraints_dict)
        self.j = len(self.joint_constraints_dict)

        self.init_big_ass_M()

        self.set_weights(self.H.weights())

        # self.set_A_hard(hard_expressions)
        # self.construct_A_soft(soft_expressions)
        self.set_A_soft(self.A.A())

        self.set_lbA(w.Matrix(self.bA.lbA()))
        self.set_ubA(w.Matrix(self.bA.ubA()))
        self.set_lb(w.Matrix(self.b.lb()))
        self.set_ub(w.Matrix(self.b.ub()))
        self.set_linear_weights(w.Matrix([0] * len(self.H.weights())))

    @profile
    def get_cmd(self, substitutions, nWSR=None):
        """
        Uses substitutions for each symbol to compute the next commands for each joint.
        :param substitutions:
        :type substitutions: list
        :return: joint name -> joint command
        :rtype: dict
        """
        np_big_ass_M = self.compiled_big_ass_M.call2(substitutions)
        np_H = np_big_ass_M[self.A_height:, :-3].copy()
        np_A = np_big_ass_M[:self.A_height, :self.A_length].copy()
        np_lb = np_big_ass_M[self.A_height:, -3].copy()
        np_ub = np_big_ass_M[self.A_height:, -2].copy()
        np_g = np_big_ass_M[self.A_height:, -1].copy()
        np_lbA = np_big_ass_M[:self.A_height, -3].copy()
        np_ubA = np_big_ass_M[:self.A_height, -2].copy()
        H, A, lb, ub, lbA, ubA, g = self.filter_zero_weight_constraints(np_H, np_A, np_lb, np_ub, np_lbA, np_ubA, np_g)
        # self.debug_print(np_H, A, lb, ub, lbA, ubA, g)
        try:
            xdot_full = self.qp_solver.solve(H, g, A, lb, ub, lbA, ubA, nWSR)
        except QPSolverException as e:
            p_weights, p_A, p_lbA, p_ubA, p_lb, p_ub = self.debug_print(np_H, A, lb, ub, lbA, ubA, g,
                                                                        actually_print=True)
            if isinstance(e, InfeasibleException):
                if self.are_joint_limits_violated(p_lb, p_ub):
                    raise OutOfJointLimitsException(e)
                raise HardConstraintsViolatedException(e)
            if isinstance(e, QPSolverException):
                arrays = [(p_weights, u'H'),
                          (p_A, u'A'),
                          (p_lbA, u'lbA'),
                          (p_ubA, u'ubA'),
                          (p_lb, u'lb'),
                          (p_ub, u'ub')]
                any_nan = False
                for a, name in arrays:
                    any_nan |= self.is_nan_in_array(name, a)
                if any_nan:
                    raise e
            raise e
        if xdot_full is None:
            return None
        # TODO enable debug print in an elegant way, preferably without slowing anything down
        self.debug_print(np_H, A, lb, ub, lbA, ubA, g, xdot_full)
        velocity = OrderedDict(
            (observable, xdot_full[i]) for i, observable in enumerate(self.joint_names()))
        acceleration = OrderedDict(
            (observable, xdot_full[i + len(self.joint_names())]) for i, observable in
            enumerate(self.joint_names()))
        jerk = OrderedDict(
            (observable, xdot_full[i + len(self.joint_names())*2]) for i, observable in
            enumerate(self.joint_names()))
        return velocity, acceleration, jerk, np_H, np_A, np_lb, np_ub, np_lbA, np_ubA, xdot_full

    def joint_names(self):
        return self.A.controlled_joint_symbols()

    def b_names(self):
        return self.b.names()

    def bA_names(self):
        return self.bA.names()

    def debug_print(self, unfiltered_H, A, lb, ub, lbA, ubA, g, xdot_full=None, actually_print=False):
        import pandas as pd
        bA_mask, b_mask = make_filter_masks(unfiltered_H, self.num_joint_constraints, self.num_hard_constraints)

        b_names = np.array(self.b_names())
        bA_names = np.array(self.bA_names())
        filtered_b_names = b_names[b_mask]
        filtered_bA_names = bA_names[bA_mask]
        filtered_H = unfiltered_H[b_mask][:, b_mask]

        p_lb = pd.DataFrame(lb, filtered_b_names, [u'data'], dtype=float)
        p_ub = pd.DataFrame(ub, filtered_b_names, [u'data'], dtype=float)
        p_g = pd.DataFrame(g, filtered_b_names, [u'data'], dtype=float)
        p_lbA = pd.DataFrame(lbA, filtered_bA_names, [u'data'], dtype=float)
        p_ubA = pd.DataFrame(ubA, filtered_bA_names, [u'data'], dtype=float)
        p_weights = pd.DataFrame(unfiltered_H.dot(np.ones(unfiltered_H.shape[0])), b_names, [u'data'],
                                 dtype=float).sort_index()
        if xdot_full is not None:
            p_xdot = pd.DataFrame(xdot_full, filtered_b_names, [u'data'], dtype=float)
            Ax = np.dot(A, xdot_full)
            p_Ax = pd.DataFrame(Ax, filtered_bA_names, [u'data'], dtype=float)
            xH = np.dot((xdot_full ** 2).T, filtered_H)
            p_xH = pd.DataFrame(xH, filtered_b_names, [u'data'], dtype=float)
            p_xg = p_g * p_xdot
            xHx = np.dot(np.dot(xdot_full.T, filtered_H), xdot_full)
            # x_soft = xdot_full[len(xdot_full) - num_slack:]
            # p_lbA_minus_x = pd.DataFrame(lbA - x_soft, filtered_bA_names, [u'data'], dtype=float).sort_index()
            # p_ubA_minus_x = pd.DataFrame(ubA - x_soft, filtered_bA_names, [u'data'], dtype=float).sort_index()
        else:
            p_xdot = None

        p_A = pd.DataFrame(A, filtered_bA_names, filtered_b_names, dtype=float)
        # if self.lbAs is None:
        #     self.lbAs = p_lbA
        # else:
        #     self.lbAs = self.lbAs.T.append(p_lbA.T, ignore_index=True).T
        # self.lbAs.T[[c for c in self.lbAs.T.columns if 'dist' in c]].plot()

        # self.save_all(p_weights, p_A, p_lbA, p_ubA, p_lb, p_ub, p_xdot)
        return p_weights, p_A, p_lbA, p_ubA, p_lb, p_ub
