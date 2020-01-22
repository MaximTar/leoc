from PyQt5.QtWidgets import QGroupBox, QHBoxLayout, QVBoxLayout, QLabel, QDoubleSpinBox, QPushButton, QWidget


class AntennaControlWidget(QGroupBox):
    def __init__(self):
        super().__init__()
        # self.setTitle("Change antenna's azimuth and/or elevation offset")
        main_layout = QHBoxLayout()
        left_layout = QVBoxLayout()
        central_layout = QVBoxLayout()
        right_layout = QVBoxLayout()

        left_layout.addWidget(QLabel("Azimuth", self))
        left_layout.addWidget(QLabel("Elevation", self))
        left_layout.addWidget(QLabel("", self))

        azimuth_spinbox = QDoubleSpinBox(self)
        # TODO move to the settings
        azimuth_spinbox.setRange(-5, 5)
        azimuth_spinbox.setSingleStep(0.1)

        elevation_spinbox = QDoubleSpinBox(self)
        # TODO move to the settings
        elevation_spinbox.setRange(-5, 5)
        elevation_spinbox.setSingleStep(0.1)

        central_layout.addWidget(azimuth_spinbox)
        central_layout.addWidget(elevation_spinbox)
        central_layout.addWidget(QLabel("", self))

        azimuth_btn = QPushButton("Set", self)
        azimuth_btn.setToolTip("Set azimuth offset")
        azimuth_btn.clicked.connect(self.azimuth_btn_clicked)

        elevation_btn = QPushButton("Set", self)
        elevation_btn.setToolTip("Set elevation offset")
        elevation_btn.clicked.connect(self.elevation_btn_clicked)

        save_btn = QPushButton("Save", self)
        save_btn.setToolTip("Save changes on controller")
        save_btn.clicked.connect(self.save_btn_clicked)

        right_layout.addWidget(azimuth_btn)
        right_layout.addWidget(elevation_btn)
        right_layout.addWidget(save_btn)

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

    def azimuth_btn_clicked(self):
        # TODO
        pass

    def elevation_btn_clicked(self):
        # TODO
        pass

    def save_btn_clicked(self):
        # TODO
        pass
