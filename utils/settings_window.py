from PyQt5.QtCore import QSettings, Qt
from PyQt5.QtWidgets import QMainWindow, QWidget, QTabWidget, QGridLayout, QLabel, QDoubleSpinBox, QSpinBox, \
    QVBoxLayout, QLineEdit, QPushButton, QHBoxLayout

from utils.lines import *

# TODO get path
SETTINGS_PATH = "/home/maxtar/satellite/AnRot/resources/settings.ini"


class SettingsWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Settings")
        # self.setFocusPolicy(Qt.StrongFocus)
        self.setWindowModality(Qt.ApplicationModal)

        self.settings = QSettings(SETTINGS_PATH, QSettings.IniFormat)

        main_vbox_layout = QVBoxLayout()
        tabs = QTabWidget()

        # first tab (general)
        general_tab = QWidget()
        general_layout = QGridLayout()

        tabs.addTab(general_tab, "General")
        self.settings.beginGroup("general")

        # left column
        general_layout.addWidget(QLabel("Observer position:", self), 0, 0, 1, -1)
        general_layout.addWidget(QLabel("Latitude", self), 1, 0)
        general_layout.addWidget(QLabel("Longitude", self), 2, 0)
        general_layout.addWidget(QLabel("Altitude", self), 3, 0)
        general_layout.addWidget(HLine(), 4, 0, 1, -1)
        general_layout.addWidget(QLabel("Update rate", self), 5, 0)

        # fillers
        general_layout.addWidget(QLabel("", self), 6, 0)
        general_layout.addWidget(QLabel("", self), 7, 0)
        general_layout.addWidget(QLabel("", self), 8, 0)

        # TODO ranges
        latitude_spinbox = QDoubleSpinBox(self)
        # azimuth_spinbox.setRange(azimuth_spinbox_range_min, azimuth_spinbox_range_max)
        latitude_spinbox.setValue(self.settings.value("observer_latitude", type=float))

        longitude_spinbox = QDoubleSpinBox(self)
        longitude_spinbox.setValue(self.settings.value("observer_longitude", type=float))

        altitude_spinbox = QDoubleSpinBox(self)
        altitude_spinbox.setValue(self.settings.value("observer_altitude", type=float))

        upd_rate_spinbox = QSpinBox(self)
        upd_rate_spinbox.setValue(self.settings.value("map_update_period", type=int))

        # right column
        general_layout.addWidget(latitude_spinbox, 1, 2)
        general_layout.addWidget(longitude_spinbox, 2, 2)
        general_layout.addWidget(altitude_spinbox, 3, 2)
        general_layout.addWidget(upd_rate_spinbox, 5, 2)

        self.settings.endGroup()
        general_tab.setLayout(general_layout)

        # second tab (antenna video)
        antenna_video_tab = QWidget()
        antenna_video_layout = QGridLayout()

        tabs.addTab(antenna_video_tab, "Antenna Video")
        self.settings.beginGroup("antenna_video")

        # left column
        antenna_video_layout.addWidget(QLabel("Camera address", self), 0, 0)
        antenna_video_layout.addWidget(HLine(), 1, 0, 1, -1)
        antenna_video_layout.addWidget(QLabel("Video widget parameters:", self), 2, 0, 1, -1)
        antenna_video_layout.addWidget(QLabel("Min width", self), 3, 0)
        antenna_video_layout.addWidget(QLabel("Min height", self), 4, 0)

        # fillers
        antenna_video_layout.addWidget(QLabel("", self), 5, 0)
        antenna_video_layout.addWidget(QLabel("", self), 6, 0)
        antenna_video_layout.addWidget(QLabel("", self), 7, 0)
        antenna_video_layout.addWidget(QLabel("", self), 8, 0)

        # TODO ranges
        camera_address_line_edit = QLineEdit(self)
        camera_address_line_edit.setText(self.settings.value("address", type=str))

        min_width_spinbox = QSpinBox(self)
        min_width_spinbox.setValue(self.settings.value("min_width", type=int))

        min_height_spinbox = QSpinBox(self)
        min_height_spinbox.setValue(self.settings.value("min_height", type=int))

        # right column
        antenna_video_layout.addWidget(camera_address_line_edit, 0, 2)
        antenna_video_layout.addWidget(min_width_spinbox, 3, 2)
        antenna_video_layout.addWidget(min_height_spinbox, 4, 2)

        self.settings.endGroup()
        antenna_video_tab.setLayout(antenna_video_layout)

        # third tab (antenna control)
        antenna_control_tab = QWidget()
        antenna_control_layout = QGridLayout()

        tabs.addTab(antenna_control_tab, "Antenna Control")
        self.settings.beginGroup("antenna_control")

        # left column
        antenna_control_layout.addWidget(QLabel("Azimuth spinbox:", self), 0, 0, 1, -1)
        antenna_control_layout.addWidget(QLabel("Range min", self), 1, 0)
        antenna_control_layout.addWidget(QLabel("Range max", self), 2, 0)
        antenna_control_layout.addWidget(QLabel("Step", self), 3, 0)
        antenna_control_layout.addWidget(HLine(), 4, 0, 1, -1)
        antenna_control_layout.addWidget(QLabel("Elevation spinbox:", self), 5, 0, 1, -1)
        antenna_control_layout.addWidget(QLabel("Range min", self), 6, 0)
        antenna_control_layout.addWidget(QLabel("Range max", self), 7, 0)
        antenna_control_layout.addWidget(QLabel("Step", self), 8, 0)

        # TODO ranges
        azimuth_range_min_spinbox = QDoubleSpinBox(self)
        # azimuth_spinbox.setRange(azimuth_spinbox_range_min, azimuth_spinbox_range_max)
        azimuth_range_min_spinbox.setValue(self.settings.value("azimuth_spinbox_range_min", type=float))

        azimuth_range_max_spinbox = QDoubleSpinBox(self)
        azimuth_range_max_spinbox.setValue(self.settings.value("azimuth_spinbox_range_max", type=float))

        azimuth_step_spinbox = QDoubleSpinBox(self)
        azimuth_step_spinbox.setValue(self.settings.value("azimuth_spinbox_step", type=float))

        elevation_range_min_spinbox = QDoubleSpinBox(self)
        # elevation_spinbox.setRange(elevation_spinbox_range_min, elevation_spinbox_range_max)
        elevation_range_min_spinbox.setValue(self.settings.value("elevation_spinbox_range_min", type=float))

        elevation_range_max_spinbox = QDoubleSpinBox(self)
        elevation_range_max_spinbox.setValue(self.settings.value("elevation_spinbox_range_max", type=float))

        elevation_step_spinbox = QDoubleSpinBox(self)
        elevation_step_spinbox.setValue(self.settings.value("elevation_spinbox_step", type=float))

        # right column
        antenna_control_layout.addWidget(azimuth_range_min_spinbox, 1, 2)
        antenna_control_layout.addWidget(azimuth_range_max_spinbox, 2, 2)
        antenna_control_layout.addWidget(azimuth_step_spinbox, 3, 2)
        antenna_control_layout.addWidget(elevation_range_min_spinbox, 6, 2)
        antenna_control_layout.addWidget(elevation_range_max_spinbox, 7, 2)
        antenna_control_layout.addWidget(elevation_step_spinbox, 8, 2)

        self.settings.endGroup()
        antenna_control_tab.setLayout(antenna_control_layout)

        # TODO connect actions
        cancel_btn = QPushButton("Cancel", self)
        cancel_btn.clicked.connect(self.cancel_btn_clicked)
        apply_btn = QPushButton("Apply", self)
        apply_btn.clicked.connect(self.apply_btn_clicked)

        buttons_hbox_layout = QHBoxLayout()
        buttons_hbox_layout.addWidget(cancel_btn)
        buttons_hbox_layout.addWidget(apply_btn)
        buttons_widget = QWidget()
        buttons_widget.setLayout(buttons_hbox_layout)

        main_vbox_layout.addWidget(tabs)
        main_vbox_layout.addWidget(buttons_widget)

        central_widget = QWidget()
        central_widget.setLayout(main_vbox_layout)
        self.setCentralWidget(central_widget)

        # empirical numbers
        self.resize(360, 10)

    def cancel_btn_clicked(self):
        self.close()

    def apply_btn_clicked(self):
        # TODO
        self.settings.sync()
        pass

    def check_file(self):
        # TODO? AFTER
        pass
