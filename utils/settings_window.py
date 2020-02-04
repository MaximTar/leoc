from PyQt5.QtCore import QSettings, Qt
from PyQt5.QtWidgets import QMainWindow, QWidget, QTabWidget, QGridLayout, QLabel, QDoubleSpinBox, QSpinBox, \
    QVBoxLayout, QLineEdit, QPushButton, QHBoxLayout

from utils.lines import *
import os

SETTINGS_PATH = os.path.dirname(os.path.abspath(__file__)) + "/../resources/settings.ini"

AZIMUTH_SPINBOX_RANGE_MIN = -6
AZIMUTH_SPINBOX_RANGE_MAX = 6
AZIMUTH_STEP_SPINBOX_RANGE_MAX = 1
ELEVATION_SPINBOX_RANGE_MIN = -6
ELEVATION_SPINBOX_RANGE_MAX = 6
ELEVATION_STEP_SPINBOX_RANGE_MAX = 1


class SettingsWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Settings")
        self.setWindowModality(Qt.ApplicationModal)

        self.settings = QSettings(SETTINGS_PATH, QSettings.IniFormat)

        main_vbox_layout = QVBoxLayout()
        self.tabs = QTabWidget()

        # first tab (general)
        general_tab = QWidget()
        general_layout = QGridLayout()

        self.tabs.addTab(general_tab, "General")
        self.settings.beginGroup("general_settings")

        # left column
        general_layout.addWidget(QLabel("Observer position:", self), 0, 0, 1, -1)
        general_layout.addWidget(QLabel("Latitude", self), 1, 0)
        general_layout.addWidget(QLabel("Longitude", self), 2, 0)
        general_layout.addWidget(QLabel("Altitude", self), 3, 0)
        general_layout.addWidget(HLine(), 4, 0, 1, -1)
        upd_lbl = QLabel("Update rate [ms]", self)
        upd_lbl.setToolTip("Map widget update rate in milliseconds")
        general_layout.addWidget(upd_lbl, 5, 0)
        general_layout.addWidget(HLine(), 6, 0, 1, -1)
        general_layout.addWidget(QLabel("Prediction:", self), 7, 0, 1, -1)
        len_lbl = QLabel("Length [h]", self)
        len_lbl.setToolTip("The elevation of horizon to compute rise time and fall time")
        general_layout.addWidget(len_lbl, 8, 0)
        general_layout.addWidget(QLabel("Horizon angle", self), 9, 0)

        # right column
        self.latitude_spinbox = QDoubleSpinBox(self)
        self.latitude_spinbox.setDecimals(6)
        self.latitude_spinbox.setRange(-90, 90)
        self.latitude_spinbox.setValue(self.settings.value("observer_latitude", type=float))

        self.longitude_spinbox = QDoubleSpinBox(self)
        self.longitude_spinbox.setDecimals(6)
        self.longitude_spinbox.setRange(-180, 180)
        self.longitude_spinbox.setValue(self.settings.value("observer_longitude", type=float))

        self.altitude_spinbox = QDoubleSpinBox(self)
        self.altitude_spinbox.setRange(-11000, 11000)
        self.altitude_spinbox.setValue(self.settings.value("observer_altitude", type=float))

        self.upd_rate_spinbox = QSpinBox(self)
        self.upd_rate_spinbox.setRange(1, 2147483644)
        self.upd_rate_spinbox.setValue(self.settings.value("map_update_period", type=int))

        self.hours_passes_spinbox = QSpinBox(self)
        self.hours_passes_spinbox.setRange(1, 2147483644)
        self.hours_passes_spinbox.setValue(self.settings.value("hours_passes", type=int))

        self.horizon_angle_spinbox = QSpinBox(self)
        self.horizon_angle_spinbox.setRange(0, 89)
        self.horizon_angle_spinbox.setValue(self.settings.value("horizon_angle", type=int))

        general_layout.addWidget(self.latitude_spinbox, 1, 2)
        general_layout.addWidget(self.longitude_spinbox, 2, 2)
        general_layout.addWidget(self.altitude_spinbox, 3, 2)
        general_layout.addWidget(self.upd_rate_spinbox, 5, 2)
        general_layout.addWidget(self.hours_passes_spinbox, 8, 2)
        general_layout.addWidget(self.horizon_angle_spinbox, 9, 2)

        self.settings.endGroup()
        general_tab.setLayout(general_layout)

        # second tab (antenna video)
        antenna_video_tab = QWidget()
        antenna_video_layout = QGridLayout()

        self.tabs.addTab(antenna_video_tab, "Antenna Video")
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
        antenna_video_layout.addWidget(QLabel("", self), 9, 0)

        # right column
        self.camera_address_line_edit = QLineEdit(self)
        self.camera_address_line_edit.setText(self.settings.value("address", type=str))

        self.min_width_spinbox = QSpinBox(self)
        self.min_width_spinbox.setRange(0, 1920)
        self.min_width_spinbox.setValue(self.settings.value("min_width", type=int))

        self.min_height_spinbox = QSpinBox(self)
        self.min_height_spinbox.setRange(0, 1080)
        self.min_height_spinbox.setValue(self.settings.value("min_height", type=int))

        antenna_video_layout.addWidget(self.camera_address_line_edit, 0, 2)
        antenna_video_layout.addWidget(self.min_width_spinbox, 3, 2)
        antenna_video_layout.addWidget(self.min_height_spinbox, 4, 2)

        self.settings.endGroup()
        antenna_video_tab.setLayout(antenna_video_layout)

        # third tab (antenna control)
        antenna_control_tab = QWidget()
        antenna_control_layout = QGridLayout()

        self.tabs.addTab(antenna_control_tab, "Antenna Control")
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

        # filler
        antenna_control_layout.addWidget(QLabel("", self), 9, 0)

        # right column
        self.azimuth_range_min_spinbox = QDoubleSpinBox(self)
        self.azimuth_range_min_spinbox.setRange(AZIMUTH_SPINBOX_RANGE_MIN, 0)
        self.azimuth_range_min_spinbox.setValue(self.settings.value("azimuth_spinbox_range_min", type=float))

        self.azimuth_range_max_spinbox = QDoubleSpinBox(self)
        self.azimuth_range_max_spinbox.setRange(0, AZIMUTH_SPINBOX_RANGE_MAX)
        self.azimuth_range_max_spinbox.setValue(self.settings.value("azimuth_spinbox_range_max", type=float))

        self.azimuth_step_spinbox = QDoubleSpinBox(self)
        self.azimuth_step_spinbox.setRange(0, AZIMUTH_STEP_SPINBOX_RANGE_MAX)
        self.azimuth_step_spinbox.setValue(self.settings.value("azimuth_spinbox_step", type=float))

        self.elevation_range_min_spinbox = QDoubleSpinBox(self)
        self.elevation_range_min_spinbox.setRange(ELEVATION_SPINBOX_RANGE_MIN, 0)
        self.elevation_range_min_spinbox.setValue(self.settings.value("elevation_spinbox_range_min", type=float))

        self.elevation_range_max_spinbox = QDoubleSpinBox(self)
        self.elevation_range_max_spinbox.setRange(0, ELEVATION_SPINBOX_RANGE_MAX)
        self.elevation_range_max_spinbox.setValue(self.settings.value("elevation_spinbox_range_max", type=float))

        self.elevation_step_spinbox = QDoubleSpinBox(self)
        self.elevation_step_spinbox.setRange(0, ELEVATION_STEP_SPINBOX_RANGE_MAX)
        self.elevation_step_spinbox.setValue(self.settings.value("elevation_spinbox_step", type=float))

        antenna_control_layout.addWidget(self.azimuth_range_min_spinbox, 1, 2)
        antenna_control_layout.addWidget(self.azimuth_range_max_spinbox, 2, 2)
        antenna_control_layout.addWidget(self.azimuth_step_spinbox, 3, 2)
        antenna_control_layout.addWidget(self.elevation_range_min_spinbox, 6, 2)
        antenna_control_layout.addWidget(self.elevation_range_max_spinbox, 7, 2)
        antenna_control_layout.addWidget(self.elevation_step_spinbox, 8, 2)

        self.settings.endGroup()
        antenna_control_tab.setLayout(antenna_control_layout)

        # buttons
        cancel_btn = QPushButton("Cancel", self)
        cancel_btn.clicked.connect(self.cancel_btn_clicked)
        apply_btn = QPushButton("Apply", self)
        apply_btn.clicked.connect(self.apply_btn_clicked)

        buttons_hbox_layout = QHBoxLayout()
        buttons_hbox_layout.addWidget(cancel_btn)
        buttons_hbox_layout.addWidget(apply_btn)
        buttons_widget = QWidget()
        buttons_widget.setLayout(buttons_hbox_layout)

        main_vbox_layout.addWidget(self.tabs)
        main_vbox_layout.addWidget(buttons_widget)

        central_widget = QWidget()
        central_widget.setLayout(main_vbox_layout)
        self.setCentralWidget(central_widget)

        # empirical numbers
        self.resize(360, 10)

    def cancel_btn_clicked(self):
        self.close()

    def apply_btn_clicked(self):
        self.settings.setValue("general_settings/observer_latitude", self.latitude_spinbox.value())
        self.settings.setValue("general_settings/observer_longitude", self.longitude_spinbox.value())
        self.settings.setValue("general_settings/observer_altitude", self.altitude_spinbox.value())
        self.settings.setValue("general_settings/map_update_period", self.upd_rate_spinbox.value())
        self.settings.setValue("general_settings/hours_passes", self.hours_passes_spinbox.value())
        self.settings.setValue("general_settings/horizon_angle", self.horizon_angle_spinbox.value())

        self.settings.setValue("antenna_video/address", self.camera_address_line_edit.text())
        self.settings.setValue("antenna_video/min_width", self.min_width_spinbox.value())
        self.settings.setValue("antenna_video/min_height", self.min_height_spinbox.value())

        self.settings.setValue("antenna_control/azimuth_spinbox_range_min", self.azimuth_range_min_spinbox.value())
        self.settings.setValue("antenna_control/azimuth_spinbox_range_max", self.azimuth_range_max_spinbox.value())
        self.settings.setValue("antenna_control/azimuth_spinbox_step", self.azimuth_step_spinbox.value())
        self.settings.setValue("antenna_control/elevation_spinbox_range_min", self.elevation_range_min_spinbox.value())
        self.settings.setValue("antenna_control/elevation_spinbox_range_max", self.elevation_range_max_spinbox.value())
        self.settings.setValue("antenna_control/elevation_spinbox_step", self.elevation_step_spinbox.value())

        # TODO refresh classes, that use these parameters
        self.settings.sync()
