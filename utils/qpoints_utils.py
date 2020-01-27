from PyQt5.QtCore import QPoint, QLineF


def qpoints_scaling(width, height, points):
    kw = width / 360
    kh = height / 180
    new_points = []
    for p in points:
        new_points.append(QPoint(p.x() * kw, p.y() * kh))
    return new_points


def qpoints_distance(pt1, pt2):
    line = QLineF(pt1, pt2)
    return line.length()


def print_array(points):
    for p in points:
        print(p.x(), p.y())
