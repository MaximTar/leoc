from datetime import datetime, timedelta

from PyQt5.QtGui import QPainter, QPainterPath
from PyQt5.QtCore import Qt, QPointF, QTimer
from PyQt5.QtWidgets import QWidget
from utils.qpoints_utils import *


# import sys  # Suppressing the error messages
#
#
# class DevNull:
#     def write(self, msg):
#         pass
#
#
# sys.stderr = DevNull()  # Suppressing the error messages


class SatelliteTrack(QWidget):
    def __init__(self, orb):
        super().__init__()
        self.orb = orb
        self.points = self.__create_points_array()
        self.no_points = False
        if self.points is None:
            self.no_points = True

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setPen(Qt.yellow)

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

    def __create_points_array(self, step_minutes=10):
        revolutions_per_day = float(self.orb.tle.mean_motion)  # mean motion
        revolution_time = 1440 / revolutions_per_day  # in minutes

        # get points for two revolutions
        points = []
        now = datetime.utcnow()
        then = now - timedelta(minutes=revolution_time)
        later = now + timedelta(minutes=revolution_time)
        while then < later:
            # noinspection PyBroadException
            try:
                lon, lat, alt = self.orb.get_lonlatalt(then)
                points.append(QPointF(lon + 180, -lat + 90))
                then += timedelta(minutes=step_minutes / revolutions_per_day)
            except BaseException as e:
                return None

        # extract points for only current revolution
        # TODO AFTER check if current position is in points
        # TODO AFTER think about 0/360 longitude
        revolution_points = []
        current = False
        for i in range(len(points) - 1):
            if current:
                revolution_points.append(points[i])
                if qpoints_distance(points[i], points[i + 1]) > 100:
                    break
            if qpoints_distance(points[i], points[i + 1]) > 100:
                current = True

        return revolution_points
