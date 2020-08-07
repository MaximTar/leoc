import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from pyqtgraph.Qt import QtCore


class AntennaGraphWidget(QWidget):
    def __init__(self):
        super().__init__()

        # TODO AFTER move numbers/colors to settings
        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')

        self.plot = pg.PlotWidget()

        self.el_r = list(range(0, 100, 10))
        ax_r = list(range(0, 100, 30))
        self.d_ax = [(r, str(r)) for r in ax_r]

        self.plot.getAxis('bottom').setTicks([self.d_ax, []])
        self.plot.getAxis('left').setTicks([self.d_ax, []])

        self.plot.addLine(x=0, pen=pg.mkPen(0.2, style=QtCore.Qt.DotLine),
                          label='90', labelOpts={'position': 0.97, 'color': 'k'})
        self.plot.addLine(x=0, pen=pg.mkPen(0.2, style=QtCore.Qt.DotLine),
                          label='270', labelOpts={'position': 0.035, 'color': 'k'})
        self.plot.addLine(y=0, pen=pg.mkPen(0.2, style=QtCore.Qt.DotLine),
                          label='0', labelOpts={'position': 0.97, 'color': 'k'})
        self.plot.addLine(y=0, pen=pg.mkPen(0.2, style=QtCore.Qt.DotLine),
                          label='180', labelOpts={'position': 0.048, 'color': 'k'})

        for r in self.el_r:
            # noinspection PyUnresolvedReferences
            circle = pg.QtGui.QGraphicsEllipseItem(-r, -r, r * 2, r * 2)
            circle.setPen(pg.mkPen(0.2, style=QtCore.Qt.DotLine))
            self.plot.addItem(circle)

        self.sat_data = self.plot.plot(pen=pg.mkPen(color='r', width=2))
        self.ant_data = self.plot.plot(pen=pg.mkPen(color='g', width=2))

        vbox_layout = QVBoxLayout()
        vbox_layout.addWidget(self.plot)
        self.setLayout(vbox_layout)

        self.is_tracking = None

    def update_sat_graph(self, azimuth, elevation):
        if self.is_tracking:
            x = elevation * np.cos(np.deg2rad(azimuth))
            y = elevation * np.sin(np.deg2rad(azimuth))
            self.sat_data.setData(x, y)

    def update_ant_graph(self, azimuth, elevation):
        # if self.is_tracking:
        x = elevation * np.cos(np.deg2rad(azimuth))
        y = elevation * np.sin(np.deg2rad(azimuth))
        self.ant_data.setData(x, y)

    def clear_sat_data(self):
        self.sat_data = self.plot.plot(pen=pg.mkPen(color='r', width=2))
