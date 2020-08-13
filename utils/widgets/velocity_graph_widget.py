import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from pyqtgraph.Qt import QtCore


class VelocityGraphWidget(QWidget):
    def __init__(self):
        super().__init__()

        # TODO AFTER move numbers/colors to settings
        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')

        self.plot = pg.PlotWidget()

        # self.plot.getPlotItem().hideAxis('bottom')
        self.plot.getPlotItem().hideAxis('left')

        self.azv_data = self.plot.plot(pen=pg.mkPen(color='r', width=2))
        self.elv_data = self.plot.plot(pen=pg.mkPen(color='g', width=2))
        self.azv_point = self.plot.plot(pen=None, symbol='o', symbolPen=None, symbolSize=6, symbolBrush='r')
        self.elv_point = self.plot.plot(pen=None, symbol='o', symbolPen=None, symbolSize=6, symbolBrush='g')

        vbox_layout = QVBoxLayout()
        vbox_layout.addWidget(self.plot)
        self.setLayout(vbox_layout)

        self.is_tracking = None

    def update_graph(self, azv_diff=None, elv_diff=None):
        x = list(range(10000))
        if azv_diff:
            # if len(diff) > 100:
            #     diff = diff[-100:]
            y = [i if i in azv_diff else 0 for i in range(1, 10000)]
            self.azv_data.setData(x, y)
            self.azv_point.setData([len(azv_diff)], [azv_diff[-1]])
        if elv_diff:
            # if len(diff) > 100:
            #     diff = diff[-100:]
            y = [i if i in elv_diff else 0 for i in range(1, 10000)]
            self.azv_data.setData(x, y)
            self.azv_point.setData([len(elv_diff)], [elv_diff[-1]])

    def clear_data(self):
        self.azv_data = self.plot.plot(pen=pg.mkPen(color='r', width=2))
        self.elv_data = self.plot.plot(pen=pg.mkPen(color='g', width=2))
