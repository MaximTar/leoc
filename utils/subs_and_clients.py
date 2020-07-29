from antenna_interfaces.msg import SatState, State
from antenna_interfaces.srv import *
from rclpy.node import Node


class SubscribersAndClients(Node):

    def __init__(self):
        super().__init__('subs_and_clients')

        self.sat_graph_slot = None
        self.ant_pose_slot = None
        self.sat_azs, self.sat_els = [], []

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

        # system
        self.sys_log_client = self.create_client(SysLog, '/antenna/sys/log')
        self.sys_auth_client = self.create_client(SysAuth, '/antenna/sys/auth')
        self.sys_deauth_client = self.create_client(SysDeauth, '/antenna/sys/deauth')

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

    def _active_satellite_state_cb(self, msg):
        self.sat_lat = msg.lat
        self.sat_lon = msg.lon
        self.sat_alt = msg.alt
        self.sat_el = msg.el
        self.sat_az = msg.az

        self.sat_azs.append(self.sat_az)
        self.sat_els.append(self.sat_el)
        if self.sat_graph_slot:
            self.sat_graph_slot(self.sat_azs, self.sat_els)

        if self.ant_pose_slot:
            self.ant_pose_slot(sat_pose=(self.sat_az, self.sat_el))

    def _antenna_state_cb(self, msg):
        self.ant_el = msg.el
        self.ant_az = msg.az
        self.ant_elv = msg.elv
        self.ant_azv = msg.azv
        # TODO HANDLE ERROR
        self.ant_err_state = msg.err_state

        if self.ant_pose_slot:
            self.ant_pose_slot(ant_pose=(self.ant_az, self.ant_el))

    def set_sat_graph_slot(self, sat_graph_slot):
        self.sat_graph_slot = sat_graph_slot

    def set_ant_pose_slot(self, ant_pose_slot):
        self.ant_pose_slot = ant_pose_slot
