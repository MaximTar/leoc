from datetime import datetime

from PyQt5.QtWidgets import QWidget, QLabel, QGridLayout, QWidgetItem

from utils.lines import *


class SatelliteDataWidget(QWidget):
    def __init__(self, settings=None):
        super().__init__()

        self.settings = None
        if settings:
            self.settings = settings

        # TODO AFTER Declination, Distance (Range), RRt, Velocity, Direction, Eclipse, VEL(OSV), Constellation

        # labels
        self.epoch_lbl = QLabel("", self)
        self.eccentricity_lbl = QLabel("", self)
        self.inclination_lbl = QLabel("", self)
        self.right_ascension_lbl = QLabel("", self)
        self.arg_perigee_lbl = QLabel("", self)
        self.mean_anomaly_lbl = QLabel("", self)
        self.mean_motion_lbl = QLabel("", self)
        self.semi_major_axis_lbl = QLabel("", self)
        self.period_lbl = QLabel("", self)
        self.perigee_lbl = QLabel("", self)
        self.sat_number_lbl = QLabel("", self)
        self.classification_lbl = QLabel("", self)
        self.id_launch_year_lbl = QLabel("", self)
        self.id_launch_number_lbl = QLabel("", self)
        self.id_launch_piece_lbl = QLabel("", self)
        self.epoch_year_lbl = QLabel("", self)
        self.epoch_day_lbl = QLabel("", self)
        self.b_star_lbl = QLabel("", self)
        self.ephemeris_type_lbl = QLabel("", self)
        self.element_number_lbl = QLabel("", self)
        self.orbit_lbl = QLabel("", self)
        self.latitude_lbl = QLabel("", self)
        self.longitude_lbl = QLabel("", self)
        self.altitude_lbl = QLabel("", self)
        self.azimuth_lbl = QLabel("", self)
        self.elevation_lbl = QLabel("", self)
        # self.right_ascension_lon_lbl = QLabel("", self)
        # self.mean_motion_derivative_lbl = QLabel("", self)
        # self.mean_motion_sec_derivative_lbl = QLabel("", self)

        self.grid_layout = QGridLayout()

        # first column
        self.grid_layout.addWidget(QLabel("Longitude", self), 0, 0)
        self.grid_layout.addWidget(QLabel("Latitude", self), 1, 0)
        self.grid_layout.addWidget(QLabel("Altitude", self), 2, 0)
        self.grid_layout.addWidget(QLabel("Azimuth", self), 3, 0)
        self.grid_layout.addWidget(QLabel("Elevation", self), 4, 0)
        self.grid_layout.addWidget(HLine(), 5, 0, 1, -1)
        self.grid_layout.addWidget(QLabel("Satellite number", self), 6, 0)
        self.grid_layout.addWidget(QLabel("Epoch", self), 7, 0)
        self.grid_layout.addWidget(QLabel("Revolution number", self), 8, 0)
        self.grid_layout.addWidget(QLabel("Inclination", self), 9, 0)
        self.grid_layout.addWidget(QLabel("Right ascension", self), 10, 0)
        self.grid_layout.addWidget(QLabel("Eccentricity", self), 11, 0)
        self.grid_layout.addWidget(QLabel("Argument of perigee", self), 12, 0)
        self.grid_layout.addWidget(QLabel("Period", self), 13, 0)
        self.grid_layout.addWidget(QLabel("Semi-major axis", self), 14, 0)
        self.grid_layout.addWidget(QLabel("Perigee", self), 15, 0)
        self.grid_layout.addWidget(QLabel("B*", self), 16, 0)
        self.grid_layout.addWidget(QLabel("Mean anomaly", self), 17, 0)
        self.grid_layout.addWidget(QLabel("Mean motion", self), 18, 0)
        self.grid_layout.addWidget(QLabel("Epoch year", self), 19, 0)
        self.grid_layout.addWidget(QLabel("Epoch day", self), 20, 0)
        self.grid_layout.addWidget(QLabel("Launch year", self), 21, 0)
        self.grid_layout.addWidget(QLabel("Launch number", self), 22, 0)
        self.grid_layout.addWidget(QLabel("Launch piece", self), 23, 0)
        self.grid_layout.addWidget(QLabel("Element number", self), 24, 0)
        self.grid_layout.addWidget(QLabel("Ephemeris type", self), 25, 0)
        self.grid_layout.addWidget(QLabel("Classification", self), 26, 0)

        # second column
        self.grid_layout.addWidget(self.longitude_lbl, 0, 2)
        self.grid_layout.addWidget(self.latitude_lbl, 1, 2)
        self.grid_layout.addWidget(self.altitude_lbl, 2, 2)
        self.grid_layout.addWidget(self.azimuth_lbl, 3, 2)
        self.grid_layout.addWidget(self.elevation_lbl, 4, 2)
        self.grid_layout.addWidget(self.sat_number_lbl, 6, 2)
        self.grid_layout.addWidget(self.epoch_lbl, 7, 2)
        self.grid_layout.addWidget(self.orbit_lbl, 8, 2)
        self.grid_layout.addWidget(self.inclination_lbl, 9, 2)
        self.grid_layout.addWidget(self.right_ascension_lbl, 10, 2)
        self.grid_layout.addWidget(self.eccentricity_lbl, 11, 2)
        self.grid_layout.addWidget(self.arg_perigee_lbl, 12, 2)
        self.grid_layout.addWidget(self.period_lbl, 13, 2)
        self.grid_layout.addWidget(self.semi_major_axis_lbl, 14, 2)
        self.grid_layout.addWidget(self.perigee_lbl, 15, 2)
        self.grid_layout.addWidget(self.b_star_lbl, 16, 2)
        self.grid_layout.addWidget(self.mean_anomaly_lbl, 17, 2)
        self.grid_layout.addWidget(self.mean_motion_lbl, 18, 2)
        self.grid_layout.addWidget(self.epoch_year_lbl, 19, 2)
        self.grid_layout.addWidget(self.epoch_day_lbl, 20, 2)
        self.grid_layout.addWidget(self.id_launch_year_lbl, 21, 2)
        self.grid_layout.addWidget(self.id_launch_number_lbl, 22, 2)
        self.grid_layout.addWidget(self.id_launch_piece_lbl, 23, 2)
        self.grid_layout.addWidget(self.element_number_lbl, 24, 2)
        self.grid_layout.addWidget(self.ephemeris_type_lbl, 25, 2)
        self.grid_layout.addWidget(self.classification_lbl, 26, 2)

        self.grid_layout.setColumnMinimumWidth(0, 150)
        self.grid_layout.setColumnMinimumWidth(2, 200)

        self.setLayout(self.grid_layout)

    # noinspection PyBroadException
    def update_data(self, orb):
        now = datetime.utcnow()

        try:
            if self.settings:
                az, el = orb.get_observer_look(now,
                                               float(self.settings.value("general_settings/observer_longitude", 0)),
                                               float(self.settings.value("general_settings/observer_latitude", 0)),
                                               float(self.settings.value("general_settings/observer_altitude", 0)))
                self.azimuth_lbl.setText(str(az))
                self.elevation_lbl.setText(str(el))
        except Exception:
            self.azimuth_lbl.setText("No data")
            self.elevation_lbl.setText("No data")

        try:
            lon, lat, alt = orb.get_lonlatalt(now)
            self.longitude_lbl.setText(str(lon))
            self.latitude_lbl.setText(str(lat))
            self.altitude_lbl.setText(str(alt))
        except Exception:
            self.longitude_lbl.setText("No data")
            self.latitude_lbl.setText("No data")
            self.altitude_lbl.setText("No data")

        self.epoch_lbl.setText(str(orb.orbit_elements.epoch))
        self.eccentricity_lbl.setText(str(orb.orbit_elements.excentricity))
        self.inclination_lbl.setText(str(orb.orbit_elements.inclination))
        self.arg_perigee_lbl.setText(str(orb.orbit_elements.arg_perigee))
        self.mean_anomaly_lbl.setText(str(orb.orbit_elements.mean_anomaly))
        self.semi_major_axis_lbl.setText(str(orb.orbit_elements.semi_major_axis))
        self.period_lbl.setText(str(orb.orbit_elements.period))
        self.perigee_lbl.setText(str(orb.orbit_elements.perigee))
        self.sat_number_lbl.setText(str(orb.tle.satnumber))
        self.classification_lbl.setText(str(orb.tle.classification))
        self.id_launch_year_lbl.setText(str(orb.tle.id_launch_year))
        self.id_launch_number_lbl.setText(str(orb.tle.id_launch_number))
        self.id_launch_piece_lbl.setText(str(orb.tle.id_launch_piece))
        self.b_star_lbl.setText(str(orb.tle.bstar))
        self.ephemeris_type_lbl.setText(str(orb.tle.ephemeris_type))
        self.element_number_lbl.setText(str(orb.tle.element_number))
        self.orbit_lbl.setText(str(orb.tle.orbit))
        self.right_ascension_lbl.setText(str(orb.orbit_elements.right_ascension))
        self.mean_motion_lbl.setText(str(orb.orbit_elements.mean_motion))
        self.epoch_year_lbl.setText(str(orb.tle.epoch_year))
        self.epoch_day_lbl.setText(str(orb.tle.epoch_day))

        # self.right_ascension_lon_lbl.setText(str(orb.orbit_elements.right_ascension_lon)
        # self.mean_motion_derivative_lbl.setText(str(orb.tle.mean_motion_derivative)
        # self.mean_motion_sec_derivative_lbl.setText(str(orb.tle.mean_motion_sec_derivative)
