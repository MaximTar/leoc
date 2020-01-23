from PyQt5.QtWidgets import QGridLayout, QGroupBox, QLabel, QDoubleSpinBox, QPushButton

from parameters import *

class AntennaControlWidget(QGroupBox):
    def __init__(self):
        super().__init__()
        # self.setTitle("Change antenna's azimuth and/or elevation offset")

        main_layout = QGridLayout()

        # left column
        main_layout.addWidget(QLabel("Azimuth", self), 0, 0)
        main_layout.addWidget(QLabel("Elevation", self), 1, 0)

        azimuth_spinbox = QDoubleSpinBox(self)
        azimuth_spinbox.setRange(azimuth_spinbox_range_min, azimuth_spinbox_range_max)
        azimuth_spinbox.setSingleStep(azimuth_spinbox_step)

        elevation_spinbox = QDoubleSpinBox(self)
        elevation_spinbox.setRange(elevation_spinbox_range_min, elevation_spinbox_range_max)
        elevation_spinbox.setSingleStep(elevation_spinbox_step)

        # central column
        main_layout.addWidget(azimuth_spinbox, 0, 2)
        main_layout.addWidget(elevation_spinbox, 1, 2)

        azimuth_btn = QPushButton("Set", self)
        azimuth_btn.setToolTip("Set azimuth offset")
        azimuth_btn.clicked.connect(self.azimuth_btn_clicked)

        elevation_btn = QPushButton("Set", self)
        elevation_btn.setToolTip("Set elevation offset")
        elevation_btn.clicked.connect(self.elevation_btn_clicked)

        save_btn = QPushButton("Save", self)
        save_btn.setToolTip("Save changes on controller")
        save_btn.clicked.connect(self.save_btn_clicked)

        # right column
        main_layout.addWidget(azimuth_btn, 0, 4)
        main_layout.addWidget(elevation_btn, 1, 4)
        main_layout.addWidget(save_btn, 2, 4)

        self.setLayout(main_layout)

    def azimuth_btn_clicked(self):
        # TODO INTERACTION
        pass

    def elevation_btn_clicked(self):
        # TODO INTERACTION
        pass

    def save_btn_clicked(self):
        # TODO INTERACTION
        pass
