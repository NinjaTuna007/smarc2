#!/usr/bin/python3

import numpy as np
import math

import tf2_geometry_msgs.tf2_geometry_msgs
from tf_transformations import euler_from_quaternion

from geometry_msgs.msg import PoseStamped, TransformStamped

from smarc_control_msgs.msg import ControlError, ControlInput, ControlReference, ControlState

from .ParamUtils import DivingModelParam
from .IDiveView import MissionStates


class PIDControl:
    """
    From:  https://github.com/DoernerD/Python_Control/blob/main/src/PID_control.py
    To simplify the PIDModel by using this.
    """
    # TODO: Add anti-windup + saturations
    def __init__(self, Kp=1.0, Ki=0.1, Kd=2.0, Kaw=0.0, u_neutral = 0.0, u_min = -1.0, u_max = 1.0):
        self._Kp = Kp
        self._Ki = Ki
        self._Kd = Kd
        self._Kaw = Kaw
        self._u_neutral = u_neutral
        self._u_min = u_min
        self._u_max = u_max

        # This also initializes all control variables
        self.reset()

    def get_control(self, x:float, x_ref:float, dt:float):
        """
        Returns the control input to the system given its current state x, its desired state x_ref
        and the time since last update dt in seconds.
        x and x_ref have to be 1 dimensional floats of the same quantity (e.g. current and desired
        heading angle)
        """
        self._error_prev = self._error
        self._error = x_ref - x
        self._integral = self._integral + self._error*dt
        self._derivative = (self._error - self._error_prev)/dt

        _u = self._Kp*self._error + self._Ki*(self._integral + self._anti_windup) + self._Kd*self._derivative + self._u_neutral

        u_lim = self._limit_control_action(_u, self._u_min, self._u_max)

        self._compute_anti_windup(_u, u_lim, dt)

        return u_lim, self._error, _u

    def _limit_control_action(self, u, u_min, u_max):
        """
        Take the hardware constraints into account.
        """
        u_lim = None

        if u > u_max:
            u_lim = u_max
        elif u < u_min:
            u_lim = u_min
        else:
            u_lim = u

        return u_lim

    def _compute_anti_windup(self, u, u_lim, dt):
        """
        Anti windup since we have hardware constraints as well as
        and integral part in our controller.
        """

        self._anti_windup += self._Kaw * (u_lim - u) * dt

    def reset(self):
        self._error = 0.0
        self._integral = 0.0
        self._anti_windup = 0.0
        self._derivative = 0.0
        self._error_prev = 0.0


class DiveControlModel:

    def __init__(self, node, view, controller, rate=0.1):

        self._node = node
        self._controller = controller
        self._view = view
        self._dt = rate

        self.param = DivingModelParam(self._node).get_param()

        # Convenience Topics
        self._current_state = None
        self._ref = None
        self._error = None
        self._input = None

        self._depth_vbs_pid = PIDControl(Kp = self.param['vbs_pid_kp'],
                                         Ki = self.param['vbs_pid_ki'],
                                         Kd = self.param['vbs_pid_kd'],
                                         Kaw = self.param['vbs_pid_kaw'],
                                         u_neutral = self.param['vbs_u_neutral'],
                                         u_min = self.param['vbs_u_min'],
                                         u_max = self.param['vbs_u_max'])
        self._pitch_lcg_pid = PIDControl(Kp = self.param['lcg_pid_kp'],
                                         Ki = self.param['lcg_pid_ki'],
                                         Kd = self.param['lcg_pid_kd'],
                                         Kaw = self.param['lcg_pid_kaw'],
                                         u_neutral = self.param['lcg_u_neutral'],
                                         u_min = self.param['lcg_u_min'],
                                         u_max = self.param['lcg_u_max'])
        self._pitch_tv_pid = PIDControl(Kp = self.param['tv_pid_kp'],
                                        Ki = self.param['tv_pid_ki'],
                                        Kd = self.param['tv_pid_kd'],
                                        Kaw = self.param['tv_pid_kaw'],
                                        u_neutral = self.param['tv_u_neutral'],
                                        u_min = self.param['tv_u_min'],
                                        u_max = self.param['tv_u_max'])
        self._yaw_pid = PIDControl(Kp = self.param['yaw_pid_kp'],
                                   Ki = self.param['yaw_pid_ki'],
                                   Kd = self.param['yaw_pid_kd'],
                                   Kaw = self.param['yaw_pid_kaw'],
                                   u_neutral = self.param['yaw_u_neutral'],
                                   u_min = self.param['yaw_u_min'],
                                   u_max = self.param['yaw_u_max'])

        self._loginfo("Dive Controller created")


    def _loginfo(self, s):
        self._node.get_logger().info(s)


    def _set_actuators_neutral(self):
        """
        Setting all actuators to neutral.
        """

        u_vbs_neutral = self.param['vbs_u_neutral']
        u_lcg_neutral = self.param['lcg_u_neutral']
        u_tv_hor_neutral = self.param['tv_u_neutral']
        u_tv_ver_neutral = self.param['tv_u_neutral']
        u_rpm_neutral = self.param['rpm_u_neutral']

        self._view.set_vbs(u_vbs_neutral)
        self._view.set_lcg(u_lcg_neutral)
        self._view.set_thrust_vector(u_tv_hor_neutral, -u_tv_ver_neutral)
        self._view.set_rpm(u_rpm_neutral)

        self._input = ControlInput()
        self._input.vbs = u_vbs_neutral
        self._input.lcg = u_lcg_neutral
        self._input.thrustervertical = u_tv_ver_neutral
        self._input.thrusterhorizontal = u_tv_hor_neutral
        self._input.thrusterrpm = float(u_rpm_neutral)

    def _set_actuators_emergency(self):
        """
        Setting all actuators to neutral.
        """

        u_vbs_emergency = self.param['vbs_u_emergency']
        u_lcg_emergency= self.param['lcg_u_emergency']
        u_tv_hor_emergency= self.param['tv_u_emergency']
        u_tv_ver_emergency= self.param['tv_u_emergency']
        u_rpm_emergency= self.param['rpm_u_emergency']

        self._view.set_vbs(u_vbs_emergency)
        self._view.set_lcg(u_lcg_emergency)
        self._view.set_thrust_vector(u_tv_hor_emergency, -u_tv_ver_emergency)
        self._view.set_rpm(u_rpm_emergency)

        self._input = ControlInput()
        self._input.vbs = u_vbs_emergency
        self._input.lcg = u_lcg_emergency
        self._input.thrustervertical = u_tv_ver_emergency
        self._input.thrusterhorizontal = u_tv_hor_emergency
        self._input.thrusterrpm = float(u_rpm_emergency)


    def update(self):
        """
        This is where all the magic happens.
        """
        mission_state = self._controller.get_mission_state()

        if mission_state == MissionStates.RECEIVED:
            self._loginfo("Mission Received")
            self._set_actuators_neutral()
            return

        if mission_state == MissionStates.COMPLETED:
            self._loginfo("Mission Complete")
            self._set_actuators_neutral()
            return

        if mission_state == MissionStates.EMERGENCY:
            # FIXME: This is never called when aborting the mission/being in emergency mode
            self._loginfo("Emergency mode. No controller running")
            self._set_actuators_emergency()
            return

        if mission_state == MissionStates.CANCELLED:
            self._loginfo("Mission Cancelled")
            self._set_actuators_neutral()
            return

        # Get setpoints
        depth_setpoint = self._controller.get_depth_setpoint()
        pitch_setpoint = self._controller.get_pitch_setpoint()
        dive_pitch_setpoint = self._controller.get_dive_pitch()
        heading_setpoint = self._controller.get_heading_setpoint()
        rpm_setpoint = self._controller.get_rpm_setpoint()

        # Get current states
        self._current_state = self._controller.get_states()
        current_depth = self._controller.get_depth()
        current_pitch = self._controller.get_pitch()
        current_heading = self._controller.get_heading()

        if not self._controller.has_waypoint():
            return

        if depth_setpoint is None:
            self._loginfo("No depth setpoint yet")
            return

        distance = self._controller.get_distance()
        goal_tolerance = self._controller.get_goal_tolerance()

        # Sketchy minus signs...
        depth_setpoint *= -1
        current_depth *= -1

        # Choose active vs. static diving based on dive pitch angle
        if np.abs(dive_pitch_setpoint) <= self.param['max_dive_pitch']:
            self._loginfo("Active Diving")
            pitch_setpoint = dive_pitch_setpoint

            u_rpm = rpm_setpoint
            u_vbs_raw = self.param['vbs_u_neutral']
            u_lcg_raw = self.param['lcg_u_neutral']
            u_vbs = u_vbs_raw
            u_lcg = u_lcg_raw

            u_tv_hor, yaw_error, u_tv_hor_raw = self._yaw_pid.get_control(current_heading, heading_setpoint, self._dt)
            u_tv_ver, pitch_error, u_tv_ver_raw = self._pitch_tv_pid.get_control(current_pitch, pitch_setpoint, self._dt)
            depth_error = depth_setpoint - current_depth

        else:
            self._loginfo("Static Diving")
            u_rpm = self.param['rpm_u_neutral']
            u_tv_ver_raw = self.param['tv_u_neutral']
            u_tv_hor_raw = self.param['tv_u_neutral']
            u_tv_ver = u_tv_ver_raw
            u_tv_hor = u_tv_hor_raw

            u_vbs, depth_error, u_vbs_raw = self._depth_vbs_pid.get_control(current_depth, depth_setpoint, self._dt)
            u_lcg, pitch_error, u_lcg_raw = self._pitch_lcg_pid.get_control(current_pitch, pitch_setpoint, self._dt)

            yaw_error = heading_setpoint - current_heading


        self._view.set_vbs(u_vbs)
        self._view.set_lcg(u_lcg)
        self._view.set_thrust_vector(u_tv_hor, -u_tv_ver) 
        self._view.set_rpm(u_rpm)

        # Convenience Topics
        self._ref = ControlReference()
        self._ref.z = depth_setpoint
        self._ref.pitch = pitch_setpoint

        self._error = ControlError()
        self._error.z = depth_error
        self._error.pitch = pitch_error
        self._error.yaw = yaw_error
        self._error.heading = current_heading

        self._input = ControlInput()
        self._input.vbs = u_vbs
        self._input.lcg = u_lcg
        self._input.thrustervertical = u_tv_ver
        self._input.thrusterhorizontal = u_tv_hor
        self._input.thrusterrpm = float(u_rpm)

        return


    def get_state(self):
        '''
        For the ConvenienceView
        '''
        if self._current_state is None:
            return None

        state = ControlState()
        state.pose.x = self._current_state.pose.pose.position.x
        state.pose.y = self._current_state.pose.pose.position.y
        state.pose.z = self._current_state.pose.pose.position.z

        rpy = euler_from_quaternion([
            self._current_state.pose.pose.orientation.x,
            self._current_state.pose.pose.orientation.y,
            self._current_state.pose.pose.orientation.z,
            self._current_state.pose.pose.orientation.w])

        state.pose.roll = rpy[0]
        state.pose.pitch = rpy[1]
        state.pose.yaw = rpy[2]

        # TODO: Add the velocity

        return state

    def get_ref(self):
        return self._ref

    def get_error(self):
        return self._error

    def get_input(self):
        return self._input

# TODO: Write unit tests here that do one loop of everything

