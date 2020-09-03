import os
import time

from antenna_interfaces.msg import *
from antenna_interfaces.srv import *
from rcl_interfaces.msg import Log
from rclpy.node import Node


# noinspection PyUnresolvedReferences
class SubscribersAndClients(Node):

    def __init__(self):
        super().__init__('subs_and_clients')

        self.sat_graph_slot = None
        self.ant_graph_slot = None
        self.sat_ant_pose_slot = None
        self.sat_ant_vel_slot = None
        self.sat_azs, self.sat_els = [], []
        self.ant_azs, self.ant_els = [], []

        self.status_slot = None
        self.clock_slot = None
        self.rosout_slot = None

        dir_name = os.path.dirname(os.path.realpath(__file__))
        time_str = time.strftime("%Y%m%d-%H%M%S")
        file_name = os.path.join(dir_name, "../logs/", time_str)
        self.log_file = open(file_name, 'a')

        self.antenna_state_sub = self.create_subscription(
            State,
            '/antenna/state',
            self._antenna_state_cb,
            10)
        self.ant_el, self.ant_az, self.ant_elv, self.ant_azv, self.ant_err_state = None, None, None, None, None

        self.active_satellite_state_sub = self.create_subscription(
            SatState,
            '/antenna/sat/state',
            self._active_satellite_state_cb,
            10)
        self.sat_lat, self.sat_lon, self.sat_alt, self.sat_el, self.sat_az = None, None, None, None, None

        self.heartbeat_sub = self.create_subscription(
            Heartbeat,
            '/antenna/heartbeat',
            self._heartbeat_cb,
            10)
        self.ts = None

        self.rosout_sub = self.create_subscription(
            Log,
            '/rosout',
            self._rosout_cb,
            20)
        self.stamp_s, self.stamp_ns, self.lvl, self.msg = None, None, None, None

        # system
        self.sys_log_client = self.create_client(SysLog, '/antenna/sys/log')
        self.sys_auth_client = self.create_client(SysAuth, '/antenna/sys/auth')
        self.sys_deauth_client = self.create_client(SysDeauth, '/antenna/sys/deauth')
        self.mcu_reset_client = self.create_client(McuReset, '/antenna/mcu/reset')
        self.sys_ping_client = self.create_client(SysPing, '/antenna/sys/ping')
        self.sys_passwd_client = self.create_client(SysPasswd, '/antenna/sys/passwd')

        # user data
        self.usr_tle_set_client = self.create_client(TlesUserSet, '/antenna/tles/user/set')
        self.usr_tle_client = self.create_client(TlesUser, '/antenna/tles/user')

        # work with satellites
        self.sat_add_client = self.create_client(SatsAdd, '/antenna/sats/add')
        self.sat_upd_client = self.create_client(SatsUpdate, '/antenna/sats/update')
        self.sat_del_client = self.create_client(SatsDel, '/antenna/sats/del')
        self.sat_names_client = self.create_client(SatsNames, '/antenna/sats/names')
        self.sat_tles_client = self.create_client(SatsTles, '/antenna/sats/tles')
        self.sat_set_active_client = self.create_client(SatsActiveSet, '/antenna/sats/active/set')
        self.sat_active_client = self.create_client(SatsActive, '/antenna/sats/active')
        self.sat_predict_client = self.create_client(SatsPredict, '/antenna/sats/predict')

        # parameters
        self.params_info_client = self.create_client(ParamsInfo, '/antenna/params/info')
        self.params_client = self.create_client(Params, '/antenna/params')
        self.params_set_client = self.create_client(ParamsSet, '/antenna/params/set')

    def _rosout_cb(self, msg):
        self.stamp_s = msg.stamp.sec
        self.stamp_ns = msg.stamp.nanosec
        self.lvl = msg.level
        self.msg = msg.msg

        line = "[{}] [{}.{}] {}\n".format(self.lvl, self.stamp_s, self.stamp_ns, self.msg)
        self.log_file.write(line)

        if self.rosout_slot:
            self.rosout_slot(self.stamp_s, self.stamp_ns, self.lvl, self.msg)

    def _active_satellite_state_cb(self, msg):
        self.sat_lat = msg.lat
        self.sat_lon = msg.lon
        self.sat_alt = msg.alt
        self.sat_el = msg.el
        self.sat_az = msg.az
        self.sat_elv = msg.elv
        self.sat_azv = msg.azv

        if self.sat_el < 0:
            self.sat_azs, self.sat_els = [], []
        else:
            self.sat_azs.append(self.sat_az)
            self.sat_els.append(self.sat_el)

        if self.sat_graph_slot:
            self.sat_graph_slot(self.sat_azs, self.sat_els)

        if self.sat_ant_pose_slot:
            self.sat_ant_pose_slot(sat_pose=(self.sat_az, self.sat_el))

        if self.sat_ant_vel_slot:
            self.sat_ant_vel_slot(sat_vel=(self.sat_azv, self.sat_elv))

    def _antenna_state_cb(self, msg):
        self.ant_el = msg.el
        self.ant_az = msg.az
        self.ant_elv = msg.elv
        self.ant_azv = msg.azv
        self.ant_err_state = msg.err_state

        if self.ant_el < 0:
            self.ant_azs, self.ant_els = [], []
        else:
            self.ant_azs.append(self.ant_az)
            self.ant_els.append(self.ant_el)

        if self.ant_graph_slot:
            self.ant_graph_slot(self.ant_azs[1:], self.ant_els[1:])

        if self.sat_ant_pose_slot:
            self.sat_ant_pose_slot(ant_pose=(self.ant_az, self.ant_el))

        if self.sat_ant_vel_slot:
            self.sat_ant_vel_slot(ant_vel=(self.ant_azv, self.ant_elv))

        if self.status_slot:
            self.status_slot(f'{self.ant_err_state:014b}'[::-1])

    def _heartbeat_cb(self, msg):
        self.ts = msg.ts
        if self.clock_slot:
            self.clock_slot(self.ts)

    def set_sat_graph_slot(self, sat_graph_slot):
        self.sat_graph_slot = sat_graph_slot

    def set_ant_graph_slot(self, ant_graph_slot):
        self.ant_graph_slot = ant_graph_slot

    def set_ant_pose_slot(self, ant_pose_slot):
        self.sat_ant_pose_slot = ant_pose_slot

    def set_ant_vel_slot(self, ant_vel_slot):
        self.sat_ant_vel_slot = ant_vel_slot

    def set_status_slot(self, status_slot):
        self.status_slot = status_slot

    def set_clock_slot(self, clock_slot):
        self.clock_slot = clock_slot

    def set_rosout_slot(self, rosout_slot):
        self.rosout_slot = rosout_slot
