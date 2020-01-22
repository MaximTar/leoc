from PyQt5.QtWidgets import QWidget, QLabel, QHBoxLayout, QVBoxLayout, QGroupBox


class AntennaPosVelWidget(QGroupBox):
    def __init__(self):
        super().__init__()
        # self.setTitle("Antenna's desired and real poses and velocities")
        main_layout = QHBoxLayout()
        left_layout = QVBoxLayout()
        central_layout = QVBoxLayout()
        right_layout = QVBoxLayout()

        left_layout.addWidget(QLabel("", self))
        left_layout.addWidget(QLabel("Pose", self))
        left_layout.addWidget(QLabel("Velocity", self))

        self.des_pose_label = QLabel("", self)
        self.des_vel_label = QLabel("", self)
        central_layout.addWidget(QLabel("Desired", self))
        central_layout.addWidget(self.des_pose_label)
        central_layout.addWidget(self.des_vel_label)

        self.real_pose_label = QLabel("", self)
        self.real_vel_label = QLabel("", self)
        right_layout.addWidget(QLabel("Real", self))
        right_layout.addWidget(self.real_pose_label)
        right_layout.addWidget(self.real_vel_label)

        left_widget = QWidget()
        central_widget = QWidget()
        right_widget = QWidget()
        left_widget.setLayout(left_layout)
        central_widget.setLayout(central_layout)
        right_widget.setLayout(right_layout)
        main_layout.addWidget(left_widget)
        main_layout.addWidget(central_widget)
        main_layout.addWidget(right_widget)

        self.setLayout(main_layout)
        # TODO update_methods
