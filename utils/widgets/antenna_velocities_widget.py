from PyQt5.QtCore import QTimer, QElapsedTimer
from PyQt5.QtWidgets import QGridLayout, QGroupBox, QLabel


# TODO LOG ALL SATELLITE AND ANTENNA DATA (+DIFF / +TIME)
# noinspection PyUnresolvedReferences
class AntennaVelocitiesWidget(QGroupBox):
    def __init__(self):
        super().__init__()
        self.setTitle("Velocity")

        main_layout = QGridLayout()

        # first column
        main_layout.addWidget(QLabel("Azimuth", self), 1, 0)
        main_layout.addWidget(QLabel("Elevation", self), 2, 0)

        self.sat_azv_label = QLabel("", self)
        self.sat_elv_label = QLabel("", self)

        # second column
        main_layout.addWidget(QLabel("Satellite", self), 0, 2)
        main_layout.addWidget(self.sat_azv_label, 1, 2)
        main_layout.addWidget(self.sat_elv_label, 2, 2)

        self.ant_azv_label = QLabel("", self)
        self.ant_elv_label = QLabel("", self)

        # third column
        main_layout.addWidget(QLabel("Antenna", self), 0, 4)
        main_layout.addWidget(self.ant_azv_label, 1, 4)
        main_layout.addWidget(self.ant_elv_label, 2, 4)

        self.diff_azv_label = QLabel("", self)
        self.diff_elv_label = QLabel("", self)

        # fourth column
        main_layout.addWidget(QLabel("Diff", self), 0, 6)
        main_layout.addWidget(self.diff_azv_label, 1, 6)
        main_layout.addWidget(self.diff_elv_label, 2, 6)

        self.setLayout(main_layout)

        self.vel_graph_slot = None

        # self.timer = QTimer()
        # self.timer.timeout.connect(self.clear_labels)

    def update_velocity(self, sat_vel=None, ant_vel=None):
        if sat_vel:
            self.sat_azv_label.setText("{:.2f}".format(sat_vel[0]))
            self.sat_elv_label.setText("{:.2f}".format(sat_vel[1]))
        if ant_vel:
            self.ant_azv_label.setText("{:.2f}".format(ant_vel[0]))
            self.ant_elv_label.setText("{:.2f}".format(ant_vel[1]))
        if self.ant_azv_label.text() and self.sat_azv_label.text():
            azv_diff = float(self.sat_azv_label.text()) - float(self.ant_azv_label.text())
            self.diff_azv_label.setText("{:.2f}".format(azv_diff))
            if self.vel_graph_slot:
                self.vel_graph_slot(azv_diff=azv_diff)
        if self.ant_elv_label.text() and self.sat_elv_label.text():
            elv_diff = float(self.sat_elv_label.text()) - float(self.ant_elv_label.text())
            self.diff_elv_label.setText("{:.2f}".format(elv_diff))
            if self.vel_graph_slot:
                self.vel_graph_slot(elv_diff=elv_diff)

    def clear_labels(self):
        self.sat_azv_label.setText(str(""))
        self.sat_elv_label.setText(str(""))
        self.ant_azv_label.setText(str(""))
        self.ant_elv_label.setText(str(""))
        self.diff_azv_label.setText(str(""))
        self.diff_elv_label.setText(str(""))

    def set_vel_graph_slot(self, vel_graph_slot):
        self.vel_graph_slot = vel_graph_slot
