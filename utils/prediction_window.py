import datetime

import rclpy
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QMainWindow, QTableWidget, QTableWidgetItem, QLabel
from antenna_interfaces.srv import SatsPredict

from utils.srv_client_handler import srv_ready


# noinspection PyUnresolvedReferences
class PredictionWindow(QMainWindow):
    def __init__(self, subs_and_clients, start_hours, length, horizon, parent=None):
        super().__init__(parent=parent)
        self.setWindowTitle("Prediction")
        self.setWindowModality(Qt.ApplicationModal)

        if srv_ready(subs_and_clients.sat_predict_client):
            req = SatsPredict.Request()
            req.hours0 = start_hours
            req.hours = length
            req.horizon = horizon
            future = subs_and_clients.sat_predict_client.call_async(req)
            while rclpy.ok():
                # TODO LOADING
                if future.done():
                    try:
                        response = future.result()
                    except Exception as e:
                        table_widget = QLabel("Cannot make prediction for satellite\n"
                                              "Stacktrace: {}".format(e), self)
                        table_widget.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
                    else:
                        if response.r_stamps:
                            table_widget = QTableWidget(self)
                            table_widget.setColumnCount(3)
                            rows = len(response.r_stamps)
                            table_widget.setRowCount(rows)
                            table_widget.setHorizontalHeaderLabels(["Rise", "Fall", "Max elevation"])
                            table_widget.horizontalHeaderItem(0).setToolTip("UTC time")
                            table_widget.horizontalHeaderItem(1).setToolTip("UTC time")
                            table_widget.horizontalHeaderItem(2).setToolTip("UTC time")
                            for i in range(0, rows):
                                r_item = QTableWidgetItem(
                                    datetime.datetime.fromtimestamp(response.r_stamps[i]).strftime("%d.%m.%Y %H:%M:%S"))
                                r_item.setFlags(Qt.ItemIsEditable)
                                table_widget.setItem(i, 0, r_item)
                                f_item = QTableWidgetItem(
                                    datetime.datetime.fromtimestamp(response.f_stamps[i]).strftime("%d.%m.%Y %H:%M:%S"))
                                f_item.setFlags(Qt.ItemIsEditable)
                                table_widget.setItem(i, 1, f_item)
                                me_item = QTableWidgetItem(
                                    datetime.datetime.fromtimestamp(response.me_stamps[i]).strftime(
                                        "%d.%m.%Y %H:%M:%S"))
                                me_item.setFlags(Qt.ItemIsEditable)
                                table_widget.setItem(i, 2, me_item)
                            table_widget.resizeColumnsToContents()
                        else:
                            table_widget = QLabel("Cannot make prediction for satellite", self)
                            table_widget.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
                    self.setCentralWidget(table_widget)
                    self.resize(500, 200)
                    break

    def centerize(self, prnt):
        if prnt:
            self.move(
                prnt.window().frameGeometry().topLeft() + prnt.window().rect().center() - self.rect().center())
