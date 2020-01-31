from PyQt5.QtWidgets import QWidget, QLabel, QGridLayout

from utils.lines import *


class SatelliteDataWidget(QWidget):
    def __init__(self):
        super().__init__()

        # label names
        # TODO Declination, Distance (Range), RRt, Velocity, Direction, Eclipse, VEL(OSV), Constellation
        # self.right_ascension_lon_name_lbl = QLabel(self)
        # self.mean_motion_derivative_name_lbl = QLabel(self)
        # self.mean_motion_sec_derivative_name_lbl = QLabel(self)

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

        grid_layout = QGridLayout()

        # first column
        grid_layout.addWidget(QLabel("Longitude", self), 0, 0)
        grid_layout.addWidget(QLabel("Latitude", self), 1, 0)
        grid_layout.addWidget(QLabel("Altitude", self), 2, 0)
        grid_layout.addWidget(QLabel("Azimuth", self), 3, 0)
        grid_layout.addWidget(QLabel("Elevation", self), 4, 0)
        grid_layout.addWidget(HLine(), 5, 0, 1, -1)
        grid_layout.addWidget(QLabel("Satellite number", self), 6, 0)
        grid_layout.addWidget(QLabel("Epoch", self), 7, 0)
        grid_layout.addWidget(QLabel("Revolution number", self), 8, 0)
        grid_layout.addWidget(QLabel("Inclination", self), 9, 0)
        grid_layout.addWidget(QLabel("Right ascension", self), 10, 0)
        grid_layout.addWidget(QLabel("Eccentricity", self), 11, 0)
        grid_layout.addWidget(QLabel("Argument of perigee", self), 12, 0)
        grid_layout.addWidget(QLabel("Period", self), 13, 0)
        grid_layout.addWidget(QLabel("Semi-major axis", self), 14, 0)
        grid_layout.addWidget(QLabel("Perigee", self), 15, 0)
        grid_layout.addWidget(QLabel("B*", self), 16, 0)
        grid_layout.addWidget(QLabel("Mean anomaly", self), 17, 0)
        grid_layout.addWidget(QLabel("Mean motion", self), 18, 0)
        grid_layout.addWidget(QLabel("Epoch year", self), 19, 0)
        grid_layout.addWidget(QLabel("Epoch day", self), 20, 0)
        grid_layout.addWidget(QLabel("Launch year", self), 21, 0)
        grid_layout.addWidget(QLabel("Launch number", self), 22, 0)
        grid_layout.addWidget(QLabel("Launch piece", self), 23, 0)
        grid_layout.addWidget(QLabel("Element number", self), 24, 0)
        grid_layout.addWidget(QLabel("Ephemeris type", self), 25, 0)
        grid_layout.addWidget(QLabel("Classification", self), 26, 0)

        # second column TODO

        self.setLayout(grid_layout)

    def update_data(self, orb):
        self.epoch_lbl.setText(orb.orbit_elements.epoch)
        self.eccentricity_lbl.setText(orb.orbit_elements.excentricity)
        self.inclination_lbl.setText(orb.orbit_elements.inclination)
        self.arg_perigee_lbl.setText(orb.orbit_elements.arg_perigee)
        self.mean_anomaly_lbl.setText(orb.orbit_elements.mean_anomaly)
        self.semi_major_axis_lbl.setText(orb.orbit_elements.semi_major_axis)
        self.period_lbl.setText(orb.orbit_elements.period)
        self.perigee_lbl.setText(orb.orbit_elements.perigee)
        self.sat_number_lbl.setText(orb.tle.satnumber)
        self.classification_lbl.setText(orb.tle.classification)
        self.id_launch_year_lbl.setText(orb.tle.id_launch_year)
        self.id_launch_number_lbl.setText(orb.tle.id_launch_number)
        self.id_launch_piece_lbl.setText(orb.tle.id_launch_piece)
        self.b_star_lbl.setText(orb.tle.bstar)
        self.ephemeris_type_lbl.setText(orb.tle.ephemeris_type)
        self.element_number_lbl.setText(orb.tle.element_number)
        self.orbit_lbl.setText(orb.tle.orbit)

        # TODO long, lat, alt, az, el
        self.right_ascension_lbl.setText(orb.orbit_elements.right_ascension)

        self.mean_motion_lbl.setText(orb.orbit_elements.mean_motion)
        # self.right_ascension_lon_lbl.setText(orb.orbit_elements.right_ascension_lon)

        self.epoch_year_lbl.setText(orb.tle.epoch_year)
        self.epoch_day_lbl.setText(orb.tle.epoch_day)
        # self.mean_motion_derivative_lbl.setText(orb.tle.mean_motion_derivative)
        # self.mean_motion_sec_derivative_lbl.setText(orb.tle.mean_motion_sec_derivative)
