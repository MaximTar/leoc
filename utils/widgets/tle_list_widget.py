import sys

import rclpy
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QListWidget, QListWidgetItem, QMessageBox, QPushButton
from antenna_interfaces.srv import *


# noinspection PyUnresolvedReferences
class TleListWidget(QListWidget):
    def __init__(self, subs_and_clients, parent_slot=None, data_slot=None):
        super().__init__()
        self.checked_indices_list = []

        self.subs_and_clients = subs_and_clients

        self.update_list(on_startup=True)

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

    def srv_ready(self, on_startup=False):
        if not self.subs_and_clients.sat_names_client.wait_for_service(timeout_sec=1.0):
            if on_startup:
                msg_box = QMessageBox()
                msg_box.setIcon(QMessageBox.Critical)
                msg_box.setText("Cannot update TLE list on startup.")
                msg_box.setWindowTitle("sat_names_client")
                ta_btn = QPushButton("Try again")
                ca_btn = QPushButton("Close app")
                msg_box.addButton(ta_btn, QMessageBox.ActionRole)
                msg_box.addButton(ca_btn, QMessageBox.ActionRole)
                msg_box.exec()
                if msg_box.clickedButton() == ta_btn:
                    self.srv_ready(on_startup)
                elif msg_box.clickedButton() == ca_btn:
                    # TODO CHECK IF WORKS
                    sys.exit()
            else:
                msg_box = QMessageBox()
                msg_box.setIcon(QMessageBox.Critical)
                msg_box.setText("Cannot update TLE list, sat_names_client is not responding.")
                msg_box.setWindowTitle("sat_names_client")
                ta_btn = QPushButton("Try again")
                ca_btn = QPushButton("Cancel")
                msg_box.addButton(ta_btn, QMessageBox.ActionRole)
                msg_box.addButton(ca_btn, QMessageBox.ActionRole)
                msg_box.exec()
                if msg_box.clickedButton() == ta_btn:
                    self.srv_ready(on_startup)
                elif msg_box.clickedButton() == ca_btn:
                    return False
        else:
            return True

    def update_list(self, on_startup=False):
        self.clear()
        # TODO AFTER think about how to save checked and selected items

        if self.srv_ready(on_startup):
            req = SatsNames.Request()
            future = self.subs_and_clients.sat_names_client.call_async(req)
            while rclpy.ok():
                # TODO LOADING?
                if future.done():
                    try:
                        response = future.result()
                    except Exception as e:
                        QMessageBox.warning(self, "sat_names_client", "Cannot update TLE list.\n"
                                                                      "Stacktrace: {}".format(e), QMessageBox.Ok)
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
