from PyQt5.QtWidgets import QGridLayout, QGroupBox, QLabel, QDoubleSpinBox, QPushButton


class AntennaControlWidget(QGroupBox):
    def __init__(self, settings):
        super().__init__()
        # self.setTitle("Change antenna's azimuth and/or elevation offset")

        main_layout = QGridLayout()

        # first row
        main_layout.addWidget(QLabel("Azimuth", self), 0, 0)
        main_layout.addWidget(QLabel("Elevation", self), 0, 1)

        # second row
        azimuth_spinbox = QDoubleSpinBox(self)
        azimuth_spinbox.setRange(-180, 180)
        azimuth_spinbox.setSingleStep(0.1)

        elevation_spinbox = QDoubleSpinBox(self)
        elevation_spinbox.setRange(0, 90)
        elevation_spinbox.setSingleStep(0.1)

        set_btn = QPushButton("Set", self)
        set_btn.setToolTip("Set antenna position")
        set_btn.clicked.connect(self.set_btn_clicked)

        main_layout.addWidget(azimuth_spinbox, 1, 0)
        main_layout.addWidget(elevation_spinbox, 1, 1)
        main_layout.addWidget(set_btn, 1, 2)

        self.setLayout(main_layout)

    def set_btn_clicked(self):
        # TODO INTERACTION
        pass
