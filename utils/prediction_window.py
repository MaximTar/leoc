import datetime

import rclpy
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QMainWindow, QTableWidget, QTableWidgetItem, QMessageBox
from antenna_interfaces.srv import SatsPredict


class PredictionWindow(QMainWindow):
    def __init__(self, subs_and_clients, start_hours, length, horizon, parent=None):
        super().__init__(parent=parent)
        self.setWindowTitle("Prediction")
        self.setWindowModality(Qt.ApplicationModal)

        req = SatsPredict.Request()
        req.hours0 = start_hours
        req.hours = length
        req.horizon = horizon
        while not subs_and_clients.sat_predict_client.wait_for_service(timeout_sec=1.0):
            # TODO LOADING
            print('Service sat_add_client is not available, waiting...')

        future = subs_and_clients.sat_predict_client.call_async(req)
        while rclpy.ok():
            # TODO LOADING
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    QMessageBox.warning(self, "sat_predict_client", "Cannot make prediction for satellite.\n"
                                                                    "Stacktrace: {}".format(e),
                                        QMessageBox.Ok)
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
                        # for r, f, me in zip(response.r_stamps, response.f_stamps, response.me_stamps):
                        for i in range(0, rows):
                            table_widget.setItem(i, 0, QTableWidgetItem(
                                datetime.datetime.fromtimestamp(response.r_stamps[i]).strftime("%d.%m.%Y %H:%M:%S")))
                            table_widget.setItem(i, 1, QTableWidgetItem(
                                datetime.datetime.fromtimestamp(response.f_stamps[i]).strftime("%d.%m.%Y %H:%M:%S")))
                            table_widget.setItem(i, 2, QTableWidgetItem(
                                datetime.datetime.fromtimestamp(response.me_stamps[i]).strftime("%d.%m.%Y %H:%M:%S")))
                        table_widget.resizeColumnsToContents()
                        self.setCentralWidget(table_widget)
                        self.resize(500, 200)
                    else:
                        QMessageBox.warning(self, "sat_predict_client",
                                            "Cannot make prediction for satellite.",
                                            QMessageBox.Ok)
                break

    def centerize(self, prnt):
        if prnt:
            self.move(
                prnt.window().frameGeometry().topLeft() + prnt.window().rect().center() - self.rect().center())
