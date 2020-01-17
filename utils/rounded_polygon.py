import math

from PyQt5.QtCore import *
from PyQt5.QtGui import QPolygon, QPainterPath

m_uiRadius = 1.0


def get_distance(pt1, pt2):
    f_d = (pt1.x() - pt2.x()) * (pt1.x() - pt2.x()) + (pt1.y() - pt2.y()) * (pt1.y() - pt2.y())
    return math.sqrt(f_d)


class RoundedPolygon(QPolygon):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def get_line_start(self, i):
        pt = QPointF()
        pt1 = self.at(i)
        pt2 = self.at((i + 1) % self.count())
        f_rat = m_uiRadius / get_distance(pt1, pt2)
        if f_rat > 0.5:
            f_rat = 0.5
        pt.setX((1.0 - f_rat) * pt1.x() + f_rat * pt2.x())
        pt.setY((1.0 - f_rat) * pt1.y() + f_rat * pt2.y())
        return pt

    def get_line_end(self, i):
        pt = QPointF()
        pt1 = self.at(i)
        pt2 = self.at((i + 1) % self.count())
        f_rat = m_uiRadius / get_distance(pt1, pt2)
        if f_rat > 0.5:
            f_rat = 0.5
        pt.setX(f_rat * pt1.x() + (1.0 - f_rat) * pt2.x())
        pt.setY(f_rat * pt1.y() + (1.0 - f_rat) * pt2.y())
        return pt

    def get_path(self):
        m_path = QPainterPath()

        for i in range(self.count()):
            pt1 = self.get_line_start(i)

            if i == 0:
                m_path.moveTo(pt1)
            else:
                m_path.quadTo(self.at(i), pt1)

            pt2 = self.get_line_end(i)
            m_path.lineTo(pt2)

        # close the last corner
        pt1 = self.get_line_start(0)
        m_path.quadTo(self.at(0), pt1)

        return m_path
