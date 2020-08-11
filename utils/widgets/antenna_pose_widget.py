from PyQt5.QtCore import QTimer, QElapsedTimer
from PyQt5.QtWidgets import QGridLayout, QGroupBox, QLabel


# TODO LOG ALL SATELLITE AND ANTENNA DATA (+DIFF / +TIME)
# noinspection PyUnresolvedReferences
class AntennaPoseWidget(QGroupBox):
    def __init__(self):
        super().__init__()
        # self.setTitle("Antenna's desired and real poses and velocities")

        main_layout = QGridLayout()

        # first column
        main_layout.addWidget(QLabel("Azimuth", self), 1, 0)
        main_layout.addWidget(QLabel("Elevation", self), 2, 0)

        self.sat_az_label = QLabel("", self)
        self.sat_el_label = QLabel("", self)

        # second column
        main_layout.addWidget(QLabel("Satellite", self), 0, 2)
        main_layout.addWidget(self.sat_az_label, 1, 2)
        main_layout.addWidget(self.sat_el_label, 2, 2)

        self.ant_az_label = QLabel("", self)
        self.ant_el_label = QLabel("", self)

        # third column
        main_layout.addWidget(QLabel("Antenna", self), 0, 4)
        main_layout.addWidget(self.ant_az_label, 1, 4)
        main_layout.addWidget(self.ant_el_label, 2, 4)

        self.diff_az_label = QLabel("", self)
        self.diff_el_label = QLabel("", self)

        # fourth column
        main_layout.addWidget(QLabel("Diff", self), 0, 6)
        main_layout.addWidget(self.diff_az_label, 1, 6)
        main_layout.addWidget(self.diff_el_label, 2, 6)

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
        if self.ant_az_label.text() and self.sat_az_label.text():
            az_diff = float(self.sat_az_label.text()) - float(self.ant_az_label.text())
            self.diff_az_label.setText("{:.2f}".format(az_diff))
        if self.ant_el_label.text() and self.sat_el_label.text():
            el_diff = float(self.sat_el_label.text()) - float(self.ant_el_label.text())
            self.diff_el_label.setText("{:.2f}".format(el_diff))

    def clear_labels(self):
        self.sat_az_label.setText(str(""))
        self.sat_el_label.setText(str(""))
        self.ant_az_label.setText(str(""))
        self.ant_el_label.setText(str(""))
        self.diff_az_label.setText(str(""))
        self.diff_el_label.setText(str(""))
