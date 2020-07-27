from rclpy.node import Node

from antenna_interfaces.srv import SysLog


class SysLogClientAsync(Node):

    def __init__(self):
        super().__init__('sys_log')
        self.cli = self.create_client(SysLog, '/antenna/sys/log')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = SysLog.Request()
        self.ok = None

    def send_request(self, log_msg):
        self.req._txt = str(log_msg)
        # self.req.txt(str(log_msg))
        self.ok = self.cli.call_async(self.req)
