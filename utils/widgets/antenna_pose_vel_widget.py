from PyQt5.QtWidgets import QGridLayout, QGroupBox, QLabel


class AntennaPosVelWidget(QGroupBox):
    def __init__(self):
        super().__init__()
        # self.setTitle("Antenna's desired and real poses and velocities")

        main_layout = QGridLayout()

        # left column
        main_layout.addWidget(QLabel("Pose", self), 1, 0)
        main_layout.addWidget(QLabel("Velocity", self), 2, 0)

        self.des_pose_label = QLabel("", self)
        self.des_vel_label = QLabel("", self)

        # central column
        main_layout.addWidget(QLabel("Desired", self), 0, 2)
        main_layout.addWidget(self.des_pose_label, 1, 2)
        main_layout.addWidget(self.des_vel_label, 2, 2)

        self.real_pose_label = QLabel("", self)
        self.real_vel_label = QLabel("", self)

        # right column
        main_layout.addWidget(QLabel("Real", self), 0, 4)
        main_layout.addWidget(self.real_pose_label, 1, 4)
        main_layout.addWidget(self.real_vel_label, 2, 4)

        self.setLayout(main_layout)
        # TODO update_methods
