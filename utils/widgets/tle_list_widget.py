import rclpy
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QListWidget, QListWidgetItem
from antenna_interfaces.srv import *


# noinspection PyUnresolvedReferences
class TleListWidget(QListWidget):
    def __init__(self, subs_and_clients, parent_slot=None, data_slot=None):
        super().__init__()
        self.checked_indices_list = []

        self.subs_and_clients = subs_and_clients

        self.update_list()

        if data_slot:
            self.itemSelectionChanged.connect(data_slot)

        self.itemChanged.connect(self.item_changed)
        if parent_slot and self.checked_indices_list:
            self.itemChanged.connect(parent_slot)

    def item_changed(self, item):
        if item.checkState() == Qt.Checked:
            self.checked_indices_list.append(self.indexFromItem(item).row())
        elif item.checkState() == Qt.Unchecked:
            try:
                self.checked_indices_list.remove(self.indexFromItem(item).row())
            except ValueError:
                pass

    def update_list(self):
        self.clear()
        # TODO AFTER think about how to save checked and selected items

        req = SatsNames.Request()
        while not self.subs_and_clients.sat_names_client.wait_for_service(timeout_sec=1.0):
            # TODO LOADING
            print('Service sat_add_client is not available, waiting...')

        future = self.subs_and_clients.sat_names_client.call_async(req)
        while rclpy.ok():
            # TODO LOADING
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    # TODO MSG_BOX
                    self.subs_and_clients.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    for i, name in zip(response.ids, response.names):
                        item = QListWidgetItem(name)
                        item.setStatusTip(str(i))
                        item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
                        item.setCheckState(Qt.Unchecked)
                        self.addItem(item)
                break

    def uncheck_item(self, item_idx):
        self.item(item_idx).setCheckState(Qt.Unchecked)
