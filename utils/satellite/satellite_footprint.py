from datetime import datetime
from math import acos, atan2, cos, degrees, radians, sin, sqrt
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QPainter, QPainterPath, QPolygon
from PyQt5.QtWidgets import QWidget
from utils.qpoints_utils import qpoints_scaling

EARTH_FLATTENING_COEFFICIENT = 0.003352891869237217
EARTH_RADIUS = 6378135


class SatelliteFootprint(QWidget):
    def __init__(self, orb):
        super().__init__()
        self.orb = orb
        self.points = self.__create_points_array()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setPen(Qt.yellow)

        scaled_points = qpoints_scaling(self.width(), self.height(), self.points)

        # TODO FIXME (when trajectory goes through edge/intersects zero)
        path = QPainterPath()
        first = True
        for p in scaled_points:
            if first:
                path.moveTo(p)
                first = False
            path.lineTo(p)

        painter.drawPolygon(QPolygon(scaled_points))
        # painter.drawPath(path)

    def __create_points_array(self):
        """
        Modified math from https://github.com/trehn/termtrack.git.
        """
        now = datetime.utcnow()
        lon, lat, alt = self.orb.get_lonlatalt(now)
        alt *= 1000
        earth_radius = earth_radius_at_latitude(lat)
        horizon_radius = acos(earth_radius / (earth_radius + alt))
        sat_footprints = []
        for hx, hy, hz in cartesian_rotation(lat, lon, horizon_radius, steps=int(self.width() / 10)):
            lat, lon = cartesian_to_latlon(hx, hy, hz)
            sat_footprints.append(QPointF(lon + 180, -lat + 90))

        return sat_footprints


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
