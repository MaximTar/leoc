from PyQt5.QtCore import QTimer, QElapsedTimer
from PyQt5.QtWidgets import QGridLayout, QGroupBox, QLabel


# noinspection PyUnresolvedReferences
class AntennaPoseWidget(QGroupBox):
    def __init__(self):
        super().__init__()
        # self.setTitle("Antenna's desired and real poses and velocities")

        main_layout = QGridLayout()

        # left column
        main_layout.addWidget(QLabel("Azimuth", self), 1, 0)
        main_layout.addWidget(QLabel("Elevation", self), 2, 0)

        self.sat_az_label = QLabel("", self)
        self.sat_el_label = QLabel("", self)

        # central column
        main_layout.addWidget(QLabel("Satellite", self), 0, 2)
        main_layout.addWidget(self.sat_az_label, 1, 2)
        main_layout.addWidget(self.sat_el_label, 2, 2)

        self.ant_az_label = QLabel("", self)
        self.ant_el_label = QLabel("", self)

        # right column
        main_layout.addWidget(QLabel("Antenna", self), 0, 4)
        main_layout.addWidget(self.ant_az_label, 1, 4)
        main_layout.addWidget(self.ant_el_label, 2, 4)

        self.setLayout(main_layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.clear_labels)

    def update_pose(self, sat_pose=None, ant_pose=None):
        if sat_pose:
            self.sat_az_label.setText("{:.2f}".format(sat_pose[0]))
            self.sat_el_label.setText("{:.2f}".format(sat_pose[1]))
        if ant_pose:
            self.ant_az_label.setText("{:.2f}".format(ant_pose[0]))
            self.ant_el_label.setText("{:.2f}".format(ant_pose[1]))

    def clear_labels(self):
        self.sat_az_label.setText(str(""))
        self.sat_el_label.setText(str(""))
        self.ant_az_label.setText(str(""))
        self.ant_el_label.setText(str(""))
