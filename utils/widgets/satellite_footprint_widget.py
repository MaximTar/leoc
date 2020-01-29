from datetime import datetime
from math import acos, atan2, cos, degrees, radians, sin, sqrt
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QPainter, QPainterPath, QPolygon
from PyQt5.QtWidgets import QWidget
from utils.qpoints_utils import *

EARTH_FLATTENING_COEFFICIENT = 0.003352891869237217
EARTH_RADIUS = 6378135


def _create_path(path, points, first=True, offset=0, full_range=True):
    if full_range:
        end = len(points)
    else:
        end = len(points) - 1
    for i in range(offset, end):
        if first:
            path.moveTo(points[i])
            first = False
        path.lineTo(points[i])
        if not full_range and qpoints_distance(points[i], points[i + 1]) > 100:
            offset = i + 1
            break
    return path, offset


class SatelliteFootprintWidget(QWidget):
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
            gap_counter = 0
            for i in range(len(self.points) - 1):
                if qpoints_distance(self.points[i], self.points[i + 1]) > 100:
                    gap_counter += 1
            scaled_points = qpoints_scaling(self.width(), self.height(), self.points)
            sorted_points = sorted(scaled_points, key=lambda point: point.x())

            # ordinary case
            if gap_counter == 0:
                painter.drawPolygon(QPolygon(scaled_points))
            # near the pole
            # TODO AFTER close and fill
            elif gap_counter == 1:
                path = QPainterPath()
                path, offset = _create_path(path, sorted_points)
                painter.drawPath(path)
            # footprint goes through edge/intersects zero
            # TODO AFTER close and fill
            elif gap_counter == 2:
                first_path = QPainterPath()
                second_path = QPainterPath()
                first_path, offset = _create_path(first_path, scaled_points, full_range=False)
                second_path, offset = _create_path(second_path, scaled_points, offset=offset, full_range=False)
                first_path, offset = _create_path(first_path, scaled_points, False, offset, True)

                painter.drawPath(first_path)
                painter.drawPath(second_path)

    def __create_points_array(self):
        """
        Modified math from https://github.com/trehn/termtrack.git.
        """
        now = datetime.utcnow()
        # noinspection PyBroadException
        try:
            lon, lat, alt = self.orb.get_lonlatalt(now)
            alt *= 1000
            earth_radius = earth_radius_at_latitude(lat)
            horizon_radius = acos(earth_radius / (earth_radius + alt))
            sat_footprints = []
            for hx, hy, hz in cartesian_rotation(lat, lon, horizon_radius, steps=int(self.width() / 10)):
                lat, lon = cartesian_to_latlon(hx, hy, hz)
                sat_footprints.append(QPointF(lon + 180, -lat + 90))

            return sat_footprints
        except BaseException:
            return None


def from_latlon(lat, lon, width, height):
    x_rel = (lon + 180) / 360
    y_rel = (-lat + 90) / 180
    x = round((width - 1) * x_rel)
    y = round((height - 1) * y_rel)
    return min(x, width - 1), min(y, height - 1)


def earth_radius_at_latitude(latitude):
    latitude = radians(abs(latitude))
    return EARTH_RADIUS * sqrt(
        1 -
        (2 * EARTH_FLATTENING_COEFFICIENT -
         (EARTH_FLATTENING_COEFFICIENT ** 2)
         )
        * (sin(latitude) ** 2)
    )


def cartesian_to_latlon(x, y, z):
    return spherical_to_latlon(*cartesian_to_spherical(x, y, z))


def cartesian_to_spherical(x, y, z):
    return acos(z), atan2(y, x)


def spherical_to_latlon(theta, phi):
    return degrees(radians(90) - theta), degrees(phi)


def latlon_to_spherical(lat, lon):
    return -radians(lat) + radians(90), radians(lon)


def spherical_to_cartesian(theta, phi):
    x = sin(theta) * cos(phi)
    y = sin(theta) * sin(phi)
    z = cos(theta)
    return x, y, z


def latlon_to_cartesian(lat, lon):
    return spherical_to_cartesian(*latlon_to_spherical(lat, lon))


def cartesian_rotation(lat, lon, r, steps=128):
    """
    Internally converts to Cartesian coordinates and applies a rotation
    matrix to yield a number of points (equal to steps) on the small
    circle described by the given latlon and radius (the latter being an
    angle as well).

    Math from https://github.com/vain/asciiworld.
    """
    # Get latitude of one point on the small circle. We can easily do
    # this by adjusting the latitude but we have to avoid pushing over a
    # pole.
    if lat > 0:
        slat = lat - degrees(r)
    else:
        slat = lat + degrees(r)

    # Geographic coordinates to spherical coordinates.
    s_theta, s_phi = latlon_to_spherical(lat, lon)

    # Cartesian coordinates of rotation axis.
    rx, ry, rz = spherical_to_cartesian(s_theta, s_phi)

    # Rotation matrix around r{x,y,z} by alpha.
    alpha = radians(360 / steps)

    m = [rx ** 2 * (1 - cos(alpha)) + cos(alpha), ry * rx * (1 - cos(alpha)) + rz * sin(alpha),
         rz * rx * (1 - cos(alpha)) - ry * sin(alpha), rx * ry * (1 - cos(alpha)) - rz * sin(alpha),
         ry ** 2 * (1 - cos(alpha)) + cos(alpha), rz * ry * (1 - cos(alpha)) + rx * sin(alpha),
         rx * rz * (1 - cos(alpha)) + ry * sin(alpha), ry * rz * (1 - cos(alpha)) - rx * sin(alpha),
         rz ** 2 * (1 - cos(alpha)) + cos(alpha)]

    # Cartesian coordinates of initial vector.
    px, py, pz = latlon_to_cartesian(slat, lon)

    for i in range(steps):
        # Rotate p{x,y,z}.
        p2x = px * m[0] + py * m[3] + pz * m[6]
        p2y = px * m[1] + py * m[4] + pz * m[7]
        p2z = px * m[2] + py * m[5] + pz * m[8]

        # For acos(), force p2z back into [-1, 1] which *might* happen
        # due to precision errors.
        p2z_fixed = max(-1, min(1, p2z))

        # Use rotated p{x,y,z} as basis for next rotation.
        px = p2x
        py = p2y
        pz = p2z

        yield p2x, p2y, p2z_fixed
