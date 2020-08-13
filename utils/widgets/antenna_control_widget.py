import rclpy
from PyQt5.QtWidgets import QGridLayout, QGroupBox, QLabel, QDoubleSpinBox, QPushButton, QMessageBox
from antenna_interfaces.srv import ParamsSet

from utils.srv_client_handler import srv_ready


# noinspection PyUnresolvedReferences
class AntennaControlWidget(QGroupBox):
    def __init__(self, subs_and_clients):
        super().__init__()
        self.subs_and_clients = subs_and_clients
        self.setTitle("Set antenna position")

        main_layout = QGridLayout()

        # first row
        main_layout.addWidget(QLabel("Azimuth", self), 0, 0)
        main_layout.addWidget(QLabel("Elevation", self), 0, 1)

        # second row
        self.azimuth_spinbox = QDoubleSpinBox(self)
        self.azimuth_spinbox.setRange(0, 359.99)
        self.azimuth_spinbox.setSingleStep(0.01)

        self.elevation_spinbox = QDoubleSpinBox(self)
        self.elevation_spinbox.setRange(0, 90)
        self.elevation_spinbox.setSingleStep(0.01)

        set_btn = QPushButton("Set", self)
        set_btn.setToolTip("Set antenna position")
        set_btn.clicked.connect(self.set_btn_clicked)

        main_layout.addWidget(self.azimuth_spinbox, 1, 0)
        main_layout.addWidget(self.elevation_spinbox, 1, 1)
        main_layout.addWidget(set_btn, 1, 2)

        self.setLayout(main_layout)

    def set_btn_clicked(self):
        if srv_ready(self.subs_and_clients.params_set_client):
            req = ParamsSet.Request()
            req.names = ["t_az", "t_el"]
            req.values = [str(self.azimuth_spinbox.value()), str(self.elevation_spinbox.value())]
            future = self.subs_and_clients.params_set_client.call_async(req)
            while rclpy.ok():
                # TODO LOADING
                if future.done():
                    try:
                        response = future.result()
                    except Exception as e:
                        QMessageBox.warning(self, "params_set_client", "Cannot change parameters.\n"
                                                                       "Stacktrace: {}".format(e), QMessageBox.Ok)
                    else:
                        if response.res:
                            QMessageBox.information(self, "params_set_client",
                                                    "Antenna moves to ({}, {}) coordinates.".format(req.values[0],
                                                                                                    req.values[1]),
                                                    QMessageBox.Ok)
                            pass
                        else:
                            QMessageBox.warning(self, "params_set_client", "Cannot change parameters.", QMessageBox.Ok)
                    break
