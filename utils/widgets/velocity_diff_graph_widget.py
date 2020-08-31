import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from pyqtgraph.Qt import QtCore


class VelocityDiffGraphWidget(QWidget):
    def __init__(self):
        super().__init__()

        # TODO AFTER move numbers/colors to settings
        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')

        self.plot = pg.PlotWidget()

        self.plot.getPlotItem().hideAxis('bottom')
        # self.plot.getPlotItem().hideAxis('left')

        self.azv_data = self.plot.plot(pen=pg.mkPen(color='b', width=2))
        self.elv_data = self.plot.plot(pen=pg.mkPen(color='m', width=2))
        self.azv_point = self.plot.plot(pen=None, symbol='o', symbolPen=None, symbolSize=6, symbolBrush='b')
        self.elv_point = self.plot.plot(pen=None, symbol='o', symbolPen=None, symbolSize=6, symbolBrush='m')

        vbox_layout = QVBoxLayout()
        vbox_layout.addWidget(self.plot)
        self.setLayout(vbox_layout)

        self.is_tracking = None

    def update_graph(self, azv_diff=None, elv_diff=None):
        # x = list(range(10000))
        # if azv_diff:
        #     # if len(diff) > 100:
        #     #     diff = diff[-100:]
        #     y = [i if i in azv_diff else 0 for i in range(1, 10001)]
        #     self.azv_data.setData(x, y)
        #     self.azv_point.setData([len(azv_diff)], [azv_diff[-1]])
        # if elv_diff:
        #     # if len(diff) > 100:
        #     #     diff = diff[-100:]
        #     y = [i if i in elv_diff else 0 for i in range(1, 10001)]
        #     self.azv_data.setData(x, y)
        #     self.azv_point.setData([len(elv_diff)], [elv_diff[-1]])
        if azv_diff:
            if len(azv_diff) > 600:
                azv_diff = azv_diff[-600:]
            # y = [i if i in az_diff else 0 for i in range(1, 101)]
            y = azv_diff
            x = list(range(len(y)))
            self.azv_data.setData(x, y)
            self.azv_point.setData([float(len(azv_diff))], [azv_diff[-1]])
        if elv_diff:
            if len(elv_diff) > 600:
                elv_diff = elv_diff[-600:]
            # y = [i if i in el_diff else 0 for i in range(1, 101)]
            y = elv_diff
            x = list(range(len(y)))
            self.elv_data.setData(x, y)
            self.elv_point.setData([len(elv_diff)], [elv_diff[-1]])

    def clear_data(self):
        self.azv_data = self.plot.plot(pen=pg.mkPen(color='b', width=2))
        self.elv_data = self.plot.plot(pen=pg.mkPen(color='m', width=2))
