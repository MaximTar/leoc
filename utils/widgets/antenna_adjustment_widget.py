import rclpy
from PyQt5.QtWidgets import QGridLayout, QGroupBox, QLabel, QDoubleSpinBox
from antenna_interfaces.srv import ParamsSet

from utils.srv_client_handler import *


# noinspection PyUnresolvedReferences
class AntennaAdjustmentWidget(QGroupBox):
    def __init__(self, settings, subs_and_clients):
        super().__init__()
        # self.setTitle("Change antenna's azimuth and/or elevation offset")
        self.subs_and_clients = subs_and_clients

        self.settings = settings
        # self.azimuth_step = float(self.settings.value("antenna_control/azimuth_spinbox_step", 0.1))
        # self.elevation_step = float(self.settings.value("antenna_control/elevation_spinbox_step", 0.1))

        self.az_corr = 0.0
        self.el_corr = 0.0

        main_layout = QGridLayout()

        # first column
        main_layout.addWidget(QLabel("Azimuth", self), 0, 0)
        main_layout.addWidget(QLabel("Elevation", self), 1, 0)

        # second column
        azimuth_minus_btn = QPushButton("-", self)
        azimuth_minus_btn.clicked.connect(self.azimuth_minus_btn_clicked)

        elevation_minus_btn = QPushButton("-", self)
        elevation_minus_btn.clicked.connect(self.elevation_minus_btn_clicked)

        main_layout.addWidget(azimuth_minus_btn, 0, 2)
        main_layout.addWidget(elevation_minus_btn, 1, 2)

        # between column
        self.azimuth_spinbox = QDoubleSpinBox(self)
        self.azimuth_spinbox.setRange(0, 5)
        self.azimuth_spinbox.setSingleStep(0.01)

        self.elevation_spinbox = QDoubleSpinBox(self)
        self.elevation_spinbox.setRange(0, 5)
        self.elevation_spinbox.setSingleStep(0.01)

        main_layout.addWidget(self.azimuth_spinbox, 0, 3)
        main_layout.addWidget(self.elevation_spinbox, 1, 3)

        # third column
        azimuth_plus_btn = QPushButton("+", self)
        azimuth_plus_btn.clicked.connect(self.azimuth_plus_btn_clicked)

        elevation_plus_btn = QPushButton("+", self)
        elevation_plus_btn.clicked.connect(self.elevation_plus_btn_clicked)

        main_layout.addWidget(azimuth_plus_btn, 0, 4)
        main_layout.addWidget(elevation_plus_btn, 1, 4)

        # # fourth column
        # save_azimuth_btn = QPushButton("Save", self)
        # save_azimuth_btn.setToolTip("Save azimuth offset")
        # save_azimuth_btn.clicked.connect(self.save_azimuth_btn_clicked)
        #
        # save_elevation_btn = QPushButton("Save", self)
        # save_elevation_btn.setToolTip("Save elevation offset")
        # save_elevation_btn.clicked.connect(self.save_elevation_btn_clicked)
        #
        # main_layout.addWidget(save_azimuth_btn, 0, 6)
        # main_layout.addWidget(save_elevation_btn, 1, 6)

        self.setLayout(main_layout)

    def azimuth_minus_btn_clicked(self):
        if srv_ready(self.subs_and_clients.params_set_client):
            self.az_corr -= self.azimuth_spinbox.value()
            req = ParamsSet.Request()
            req.names = ["az_cor"]
            req.values = [str(self.az_corr)]
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
                            # QMessageBox.information(self, "params_set_client",
                            #                         "Antenna moves to ({}, {}) coordinates.".format(req.values[0],
                            #                                                                         req.values[1]),
                            #                         QMessageBox.Ok)
                            pass
                        else:
                            QMessageBox.warning(self, "params_set_client", "Cannot change parameters.", QMessageBox.Ok)
                    break

    def azimuth_plus_btn_clicked(self):
        if srv_ready(self.subs_and_clients.params_set_client):
            self.az_corr += self.azimuth_spinbox.value()
            req = ParamsSet.Request()
            req.names = ["az_cor"]
            req.values = [str(self.az_corr)]
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
                            # QMessageBox.information(self, "params_set_client",
                            #                         "Antenna moves to ({}, {}) coordinates.".format(req.values[0],
                            #                                                                         req.values[1]),
                            #                         QMessageBox.Ok)
                            pass
                        else:
                            QMessageBox.warning(self, "params_set_client", "Cannot change parameters.", QMessageBox.Ok)
                    break

    # def save_azimuth_btn_clicked(self):
    #     # TODO INTERACTION
    #     pass

    def elevation_minus_btn_clicked(self):
        if srv_ready(self.subs_and_clients.params_set_client):
            self.el_corr -= self.azimuth_spinbox.value()
            req = ParamsSet.Request()
            req.names = ["el_cor"]
            req.values = [str(self.el_corr)]
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
                            # QMessageBox.information(self, "params_set_client",
                            #                         "Antenna moves to ({}, {}) coordinates.".format(req.values[0],
                            #                                                                         req.values[1]),
                            #                         QMessageBox.Ok)
                            pass
                        else:
                            QMessageBox.warning(self, "params_set_client", "Cannot change parameters.", QMessageBox.Ok)
                    break

    def elevation_plus_btn_clicked(self):
        if srv_ready(self.subs_and_clients.params_set_client):
            self.el_corr += self.azimuth_spinbox.value()
            req = ParamsSet.Request()
            req.names = ["el_cor"]
            req.values = [str(self.el_corr)]
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
                            # QMessageBox.information(self, "params_set_client",
                            #                         "Antenna moves to ({}, {}) coordinates.".format(req.values[0],
                            #                                                                         req.values[1]),
                            #                         QMessageBox.Ok)
                            pass
                        else:
                            QMessageBox.warning(self, "params_set_client", "Cannot change parameters.", QMessageBox.Ok)
                    break

    # def save_elevation_btn_clicked(self):
    #     # TODO INTERACTION
    #     pass
