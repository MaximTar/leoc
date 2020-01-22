from datetime import datetime
from enum import Enum

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QComboBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget

from utils.time_handler import ntplib_ntp_time, ntp_time
from utils.internet_handler import ping


class AntennaTimeWidget(QGroupBox):
    class TimeResult(Enum):
        OK = 0
        NO_CONNECTION = 1
        SERVERS_UNAVAILABLE = 2

    def __init__(self):
        super().__init__()
        self.setTitle("UTC Time:")

        self.dt = None

        # TODO (and other antenna_widgets)
        # main_layout = QGridLayout()
        main_layout = QHBoxLayout()
        left_layout = QVBoxLayout()
        right_layout = QVBoxLayout()

        self.ntp_servers_cbx = QComboBox(self)
        self.ntp_servers_cbx.addItems(["ntp1.vniiftri.ru", "ntp2.vniiftri.ru", "ntp3.vniiftri.ru", "ntp21.vniiftri.ru"])
        self.ntp_servers_cbx.currentIndexChanged.connect(self.ntp_server_changed)
        left_layout.addWidget(self.ntp_servers_cbx)
        left_layout.addWidget(QLabel("Antenna", self))
        left_layout.addWidget(QLabel("", self))

        self.ntp_time_lbl = QLabel("", self)
        self.ntp_time_lbl.setAlignment(Qt.AlignVCenter)
        antenna_time_lbl = QLabel("", self)
        sync_btn = QPushButton("Synchronize", self)
        sync_btn.setToolTip("Synchronize time")
        sync_btn.clicked.connect(self.sync_btn_clicked)

        right_layout.addWidget(self.ntp_time_lbl)
        right_layout.addWidget(antenna_time_lbl)
        right_layout.addWidget(sync_btn)

        left_widget = QWidget()
        right_widget = QWidget()
        left_widget.setLayout(left_layout)
        right_widget.setLayout(right_layout)
        main_layout.addWidget(left_widget)
        main_layout.addWidget(right_widget)

        self.setLayout(main_layout)
        self.ntp_server_changed()

    def sync_btn_clicked(self):
        # TODO
        pass

    def ntp_server_changed(self):
        t = ntplib_ntp_time(self.ntp_servers_cbx.currentText())
        if t is not None:
            self.ntp_time_lbl.setText(t.strftime("%H:%M:%S"))
        else:
            self.ntp_time_lbl.setText("Server is not available")

    def get_time_delta(self):
        if ping(self.ntp_servers_cbx.currentText()):
            t = ntplib_ntp_time(self.ntp_servers_cbx.currentText())
            if t is not None:
                self.dt = ntplib_ntp_time(self.ntp_servers_cbx.currentText()) - datetime.utcnow()
                return self.TimeResult.OK
            else:
                return self.TimeResult.SERVERS_UNAVAILABLE
        elif self.ntp_servers_cbx.currentIndex() < self.ntp_servers_cbx.count() - 1:
            self.ntp_servers_cbx.setCurrentIndex(self.ntp_servers_cbx.currentIndex() + 1)
            self.get_time_delta()
        else:
            return self.TimeResult.NO_CONNECTION
