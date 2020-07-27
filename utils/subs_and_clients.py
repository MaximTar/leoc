from antenna_interfaces.msg import SatState, State
from antenna_interfaces.srv import *
from rclpy.node import Node


class Subscribers(Node):

    def __init__(self):
        super().__init__('subs_and_clients')

        self.antenna_state_sub = self.create_subscription(
            State,
            '/antenna/state',
            self._antenna_state_cb,
            10)
        self.ant_el, self.ant_az, self.ant_elv, self.ant_azv, self.ant_err_state = None, None, None, None, None

        self.active_satellite_state_sub = self.create_subscription(
            SatState,
            '/antenna/sat/active',
            self._active_satellite_state_cb,
            10)
        self.sat_lat, self.sat_lon, self.sat_alt, self.sat_el, self.sat_az = None, None, None, None, None

        self.sys_log_client = self.create_client(SysLog, '/antenna/sys/log')
        self.sat_add_client = self.create_client(SatsAdd, '/antenna/sats/add')
        self.sat_upd_client = self.create_client(SatsUpdate, '/antenna/sats/update')
        self.sat_del_client = self.create_client(SatsDel, '/antenna/sats/del')
        self.sat_names_client = self.create_client(SatsNames, '/antenna/sats/names')
        self.sat_tles_client = self.create_client(SatsTles, '/antenna/sats/tles')
        self.sat_set_active_client = self.create_client(SatsActiveSet, '/antenna/sats/active/set')
        self.sat_active_client = self.create_client(SatsActive, '/antenna/sats/active')

        # while not self.sys_log_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service not available, waiting again...')
        # self.req = SysLog.Request()
        # self.ok = None

    # def send_request(self, log_msg):
    #     self.req._txt = str(log_msg)
    #     # self.req.txt(str(log_msg))
    #     self.ok = self.sys_log_client.call_async(self.req)

    def _active_satellite_state_cb(self, msg):
        self.lat = msg.lat
        self.lon = msg.lon
        self.alt = msg.alt
        self.el = msg.el
        self.az = msg.az

    def _antenna_state_cb(self, msg):
        self.el = msg.el
        self.az = msg.az
        self.elv = msg.elv
        self.azv = msg.azv
        self.err_state = msg.err_state
