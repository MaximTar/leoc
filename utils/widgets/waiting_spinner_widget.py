"""
https://github.com/z3ntu/QtWaitingSpinner
The MIT License (MIT)

Copyright (c) 2012-2014 Alexander Turkin
Copyright (c) 2014 William Hallatt
Copyright (c) 2015 Jacob Dawid
Copyright (c) 2016 Luca Weiss

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import math

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *


# noinspection PyUnresolvedReferences,PyTypeChecker
class WaitingSpinnerWidget(QWidget):
    def __init__(self, parent, center_on_parent=True, disable_parent_when_spinning=False, modality=Qt.NonModal):
        super().__init__(parent)

        self._centerOnParent = center_on_parent
        self._disableParentWhenSpinning = disable_parent_when_spinning

        # WAS IN initialize()
        self._color = QColor(Qt.black)
        self._roundness = 100.0
        self._minimumTrailOpacity = 3.14159265358979323846
        self._trailFadePercentage = 80.0
        self._revolutionsPerSecond = 1.57079632679489661923
        self._numberOfLines = 20
        self._lineLength = 10
        self._lineWidth = 2
        self._innerRadius = 10
        self._currentCounter = 0
        self._isSpinning = False

        self._timer = QTimer()
        self._timer.timeout.connect(self.rotate)
        self.update_size()
        self.update_timer()
        self.hide()
        # END initialize()

        self.setWindowModality(modality)
        self.setAttribute(Qt.WA_TranslucentBackground)

    def paintEvent(self, q_paint_event):
        self.update_position()
        painter = QPainter(self)
        painter.fillRect(self.rect(), Qt.transparent)
        painter.setRenderHint(QPainter.Antialiasing, True)

        if self._currentCounter >= self._numberOfLines:
            self._currentCounter = 0

        painter.setPen(Qt.NoPen)
        for i in range(0, self._numberOfLines):
            painter.save()
            painter.translate(self._innerRadius + self._lineLength, self._innerRadius + self._lineLength)
            rotate_angle = float(360 * i) / float(self._numberOfLines)
            painter.rotate(rotate_angle)
            painter.translate(self._innerRadius, 0)
            distance = self.line_count_distance_from_primary(i, self._currentCounter, self._numberOfLines)
            color = self.current_line_color(distance, self._numberOfLines, self._trailFadePercentage,
                                            self._minimumTrailOpacity, self._color)
            painter.setBrush(color)
            painter.drawRoundedRect(QRect(0, int(-self._lineWidth / 2), self._lineLength, self._lineWidth),
                                    self._roundness, self._roundness, Qt.RelativeSize)
            painter.restore()

    def start(self):
        self.update_position()
        self._isSpinning = True
        self.show()

        if self.parentWidget and self._disableParentWhenSpinning:
            self.parentWidget().setEnabled(False)

        if not self._timer.isActive():
            self._timer.start()
            self._currentCounter = 0

    def stop(self):
        self._isSpinning = False
        self.hide()

        if self.parentWidget() and self._disableParentWhenSpinning:
            self.parentWidget().setEnabled(True)

        if self._timer.isActive():
            self._timer.stop()
            self._currentCounter = 0

    def set_number_of_lines(self, lines):
        self._numberOfLines = lines
        self._currentCounter = 0
        self.update_timer()

    def set_line_length(self, length):
        self._lineLength = length
        self.update_size()

    def set_line_width(self, width):
        self._lineWidth = width
        self.update_size()

    def set_inner_radius(self, radius):
        self._innerRadius = radius
        self.update_size()

    def color(self):
        return self._color

    def roundness(self):
        return self._roundness

    def minimum_trail_opacity(self):
        return self._minimumTrailOpacity

    def trail_fade_percentage(self):
        return self._trailFadePercentage

    def revolutions_pers_second(self):
        return self._revolutionsPerSecond

    def number_of_lines(self):
        return self._numberOfLines

    def line_length(self):
        return self._lineLength

    def line_width(self):
        return self._lineWidth

    def inner_radius(self):
        return self._innerRadius

    def is_spinning(self):
        return self._isSpinning

    def set_roundness(self, roundness):
        self._roundness = max(0.0, min(100.0, roundness))

    def set_color(self, color=Qt.black):
        self._color = QColor(color)

    def set_revolutions_per_second(self, revolutions_per_second):
        self._revolutionsPerSecond = revolutions_per_second
        self.update_timer()

    def set_trail_fade_percentage(self, trail):
        self._trailFadePercentage = trail

    def set_minimum_trail_opacity(self, minimum_trail_opacity):
        self._minimumTrailOpacity = minimum_trail_opacity

    def rotate(self):
        self._currentCounter += 1
        if self._currentCounter >= self._numberOfLines:
            self._currentCounter = 0
        self.update()

    def update_size(self):
        size = (self._innerRadius + self._lineLength) * 2
        self.setFixedSize(size, size)

    def update_timer(self):
        self._timer.setInterval(1000 / (self._numberOfLines * self._revolutionsPerSecond))

    def update_position(self):
        if self.parentWidget() and self._centerOnParent:
            self.move(self.parentWidget().width() / 2 - self.width() / 2,
                      self.parentWidget().height() / 2 - self.height() / 2)

    # noinspection PyMethodMayBeStatic
    def line_count_distance_from_primary(self, current, primary, total_nr_of_lines):
        distance = primary - current
        if distance < 0:
            distance += total_nr_of_lines
        return distance

    # noinspection PyMethodMayBeStatic
    def current_line_color(self, count_distance, total_nr_of_lines, trail_fade_perc, min_opacity, color_input):
        color = QColor(color_input)
        if count_distance == 0:
            return color
        min_alpha_f = min_opacity / 100.0
        distance_threshold = int(math.ceil((total_nr_of_lines - 1) * trail_fade_perc / 100.0))
        if count_distance > distance_threshold:
            color.setAlphaF(min_alpha_f)
        else:
            alpha_diff = color.alphaF() - min_alpha_f
            gradient = alpha_diff / float(distance_threshold + 1)
            result_alpha = color.alphaF() - gradient * count_distance
            # If alpha is out of bounds, clip it.
            result_alpha = min(1.0, max(0.0, result_alpha))
            color.setAlphaF(result_alpha)
        return color
