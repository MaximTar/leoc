from datetime import datetime

import numpy as np
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QPainter, QPainterPath, QPen
from PyQt5.QtWidgets import QWidget

from utils.qpoints_utils import qpoints_scaling


def julian_day_from_date(date, calendar='standard'):
    """
    Creates a Julian Day from a 'datetime-like' object
    Returns the fractional Julian Day (resolution 1 second)
    - if calendar='standard' or 'gregorian' (default), Julian day follows Julian
    Calendar on and before 1582-10-5, Gregorian calendar after 1582-10-15
    - if calendar='proleptic_gregorian', Julian Day follows gregorian calendar
    - if calendar='julian', Julian Day follows julian calendar
    Algorithm:
    Meeus, Jean (1998) Astronomical Algorithms (2nd Edition). Willmann-Bell,
    Virginia. p. 63
    """
    # based on redate.py by David Finlayson
    year = date.year
    month = date.month
    day = date.day
    hour = date.hour
    minute = date.minute
    second = date.second
    # Convert time to fractions of a day
    day = day + hour / 24.0 + minute / 1440.0 + second / 86400.0
    # Start Meeus algorithm (variables are in his notation)
    if month < 3:
        month = month + 12
        year = year - 1
    a = int(year / 100)
    jd = int(365.25 * (year + 4716)) + int(30.6001 * (month + 1)) + day - 1524.5
    # optionally adjust the jd for the switch from
    # the Julian to Gregorian Calendar
    # here assumed to have occurred the day after 1582 October 4
    if calendar in ['standard', 'gregorian']:
        if jd >= 2299170.5:
            # 1582 October 15 (Gregorian Calendar)
            b = 2 - a + int(a / 4)
        elif jd < 2299160.5:
            # 1582 October 5 (Julian Calendar)
            b = 0
        else:
            raise ValueError(
                'impossible date (falls in gap between end of Julian calendar and beginning of Gregorian calendar')
    elif calendar == 'proleptic_gregorian':
        b = 2 - a + int(a / 4)
    elif calendar == 'julian':
        b = 0
    else:
        raise ValueError(
            'unknown calendar, must be one of julian,standard,gregorian,proleptic_gregorian, got %s' % calendar)
    # adjust for Julian calendar if necessary
    jd = jd + b
    return jd


def ephemeris(date):
    """
    input:  date - datetime object (assumed UTC)
    output: gha - Greenwich hour angle, the angle between the Greenwich
            meridian and the meridian containing the subsolar point
            dec - solar declination
    """
    deg2rad = np.pi / 180.
    rad2deg = 1. / deg2rad
    # compute julian day from UTC datetime object
    # datetime objects use proleptic gregorian calendar
    j_day = julian_day_from_date(date, calendar='proleptic_gregorian')
    jd = np.floor(j_day)  # truncate to integer
    # utc hour
    ut = date.hour + date.minute / 60. + date.second / 3600.
    # calculate number of centuries from J2000
    t = (jd + (ut / 24.) - 2451545.0) / 36525.
    # mean longitude corrected for aberration
    mean_long = (280.460 + 36000.770 * t) % 360
    # mean anomaly
    g = 357.528 + 35999.050 * t
    # ecliptic longitude
    lm = mean_long + 1.915 * np.sin(g * deg2rad) + 0.020 * np.sin(2 * g * deg2rad)
    # obliquity of the ecliptic
    ep = 23.4393 - 0.01300 * t
    # equation of time
    eq_time = -1.915 * np.sin(g * deg2rad) - 0.020 * np.sin(2 * g * deg2rad) + 2.466 * np.sin(
        2 * lm * deg2rad) - 0.053 * np.sin(4 * lm * deg2rad)
    # Greenwich hour angle
    gha = 15 * ut - 180 + eq_time
    # declination of sun
    dec = np.arcsin(np.sin(ep * deg2rad) * np.sin(lm * deg2rad)) * rad2deg
    return gha, dec


def day_night_terminator(date, delta=0.25, lon_min=-180, lon_max=180):
    """
    date is datetime object (assumed UTC)
    """
    dg2rad = np.pi / 180.
    longitudes = np.arange(lon_min, lon_max + 0.5 * delta, delta, dtype=np.float32)
    # compute greenwich hour angle and solar declination
    # from datetime object (assumed UTC)
    tau, dec = ephemeris(date)
    # compute day/night terminator from hour angle, declination
    longitude = longitudes + tau
    latitudes = np.arctan(-np.cos(longitude * dg2rad) / np.tan(dec * dg2rad)) / dg2rad
    return longitudes, latitudes


def _create_points_array(ts=None):
    now = datetime.utcfromtimestamp(ts) if ts else datetime.utcnow()
    longs, lats = day_night_terminator(now)
    coordinates = tuple(zip(longs, lats))
    points = []
    for p in coordinates:
        points.append(QPointF(p[0] + 180, -p[1] + 90))
    return points


class TerminatorWidget(QWidget):
    """
    Slightly modified code from
    https://github.com/matplotlib/basemap/tree/5f6a39f46464794dab31915a22073e07b347a331
    """

    def __init__(self, ts=None):
        super().__init__()
        self.points = _create_points_array(ts=ts)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setPen(Qt.white)

        # painter.setBrush(QBrush(Qt.red, Qt.SolidPattern))

        if self.points is not None:
            scaled_points = qpoints_scaling(self.width(), self.height(), self.points)

            path = QPainterPath()
            first = True
            for p in scaled_points:
                if first:
                    path.moveTo(p)
                    first = False
                path.lineTo(p)

            painter.drawPath(path)

            pen = QPen(Qt.red, 0, Qt.CustomDashLine)
            pen.setDashPattern([15, 15])
            painter.setPen(pen)

            painter.drawPath(path)
