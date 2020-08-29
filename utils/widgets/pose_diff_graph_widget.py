import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from pyqtgraph.Qt import QtCore


class PoseDiffGraphWidget(QWidget):
    def __init__(self):
        super().__init__()

        # TODO AFTER move numbers/colors to settings
        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')

        self.plot = pg.PlotWidget()

        self.plot.getPlotItem().hideAxis('bottom')
        # self.plot.getPlotItem().hideAxis('left')

        self.az_data = self.plot.plot(pen=pg.mkPen(color='b', width=2))
        self.el_data = self.plot.plot(pen=pg.mkPen(color='m', width=2))
        self.az_point = self.plot.plot(pen=None, symbol='o', symbolPen=None, symbolSize=6, symbolBrush='b')
        self.el_point = self.plot.plot(pen=None, symbol='o', symbolPen=None, symbolSize=6, symbolBrush='m')

        vbox_layout = QVBoxLayout()
        vbox_layout.addWidget(self.plot)
        self.setLayout(vbox_layout)

        self.resize(self.width(), 200)

        self.is_tracking = None

    def update_graph(self, az_diff=None, el_diff=None):
        # x = list(range(100))
        if az_diff:
            if len(az_diff) > 600:
                az_diff = az_diff[-600:]
            # y = [i if i in az_diff else 0 for i in range(1, 101)]
            y = az_diff
            x = list(range(len(y)))
            self.az_data.setData(x, y)
            self.az_point.setData([len(az_diff)], [az_diff[-1]])
        if el_diff:
            if len(el_diff) > 600:
                el_diff = el_diff[-600:]
            # y = [i if i in el_diff else 0 for i in range(1, 101)]
            y = el_diff
            x = list(range(len(y)))
            self.el_data.setData(x, y)
            self.el_point.setData([len(el_diff)], [el_diff[-1]])

    def clear_data(self):
        self.az_data = self.plot.plot(pen=pg.mkPen(color='b', width=2))
        self.el_data = self.plot.plot(pen=pg.mkPen(color='m', width=2))
