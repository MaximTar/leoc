from PyQt5.QtWidgets import QGridLayout, QGroupBox, QLabel, QPushButton


class AntennaAdjustmentWidget(QGroupBox):
    def __init__(self, settings):
        super().__init__()
        # self.setTitle("Change antenna's azimuth and/or elevation offset")

        self.settings = settings
        self.azimuth_step = float(self.settings.value("antenna_control/azimuth_spinbox_step", 0.1))
        self.elevation_step = float(self.settings.value("antenna_control/elevation_spinbox_step", 0.1))

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

        # third column
        azimuth_plus_btn = QPushButton("+", self)
        azimuth_plus_btn.clicked.connect(self.azimuth_plus_btn_clicked)

        elevation_plus_btn = QPushButton("+", self)
        elevation_plus_btn.clicked.connect(self.elevation_plus_btn_clicked)

        main_layout.addWidget(azimuth_plus_btn, 0, 4)
        main_layout.addWidget(elevation_plus_btn, 1, 4)

        # fourth column
        save_azimuth_btn = QPushButton("Save", self)
        save_azimuth_btn.setToolTip("Save azimuth offset")
        save_azimuth_btn.clicked.connect(self.save_azimuth_btn_clicked)

        save_elevation_btn = QPushButton("Save", self)
        save_elevation_btn.setToolTip("Save elevation offset")
        save_elevation_btn.clicked.connect(self.save_elevation_btn_clicked)

        main_layout.addWidget(save_azimuth_btn, 0, 6)
        main_layout.addWidget(save_elevation_btn, 1, 6)

        self.setLayout(main_layout)

    def update_steps(self):
        self.azimuth_step = float(self.settings.value("antenna_control/azimuth_spinbox_step", 0.1))
        self.elevation_step = float(self.settings.value("antenna_control/elevation_spinbox_step", 0.1))

    def azimuth_minus_btn_clicked(self):
        # TODO INTERACTION
        pass

    def azimuth_plus_btn_clicked(self):
        # TODO INTERACTION
        pass

    def save_azimuth_btn_clicked(self):
        # TODO INTERACTION
        pass

    def elevation_minus_btn_clicked(self):
        # TODO INTERACTION
        pass

    def elevation_plus_btn_clicked(self):
        # TODO INTERACTION
        pass

    def save_elevation_btn_clicked(self):
        # TODO INTERACTION
        pass

    # azimuth_spinbox.setRange(float(settings.value("antenna_control/azimuth_spinbox_range_min", -5)),
    #                          float(settings.value("antenna_control/azimuth_spinbox_range_max", 5)))
    # azimuth_spinbox.setSingleStep(float(settings.value("antenna_control/azimuth_spinbox_step", 0.1)))
    # elevation_spinbox.setRange(float(settings.value("antenna_control/elevation_spinbox_range_min", -5)),
    #                            float(settings.value("antenna_control/elevation_spinbox_range_max", 5)))
    # elevation_spinbox.setSingleStep(float(settings.value("antenna_control/elevation_spinbox_step", 0.1)))
