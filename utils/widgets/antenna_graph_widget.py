import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from pyqtgraph.Qt import QtCore


# TODO AFTER FULL SCREEN GRAPH
# TODO AFTER SHOW MANY SATELLITES
class AntennaGraphWidget(QWidget):
    def __init__(self):
        super().__init__()

        # TODO AFTER move numbers/colors to settings
        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')

        self.plot = pg.PlotWidget()

        el_r = list(range(0, 100, 10))

        self.plot.getPlotItem().hideAxis('bottom')
        self.plot.getPlotItem().hideAxis('left')

        # 0 N / 90 E / 180 S / 270 W
        self.plot.addLine(x=0, pen=pg.mkPen(0.2, style=QtCore.Qt.DotLine),
                          label='N', labelOpts={'position': 0.97, 'color': 'k'})
        self.plot.addLine(x=0, pen=pg.mkPen(0.2, style=QtCore.Qt.DotLine),
                          label='S', labelOpts={'position': 0.03, 'color': 'k'})
        self.plot.addLine(y=0, pen=pg.mkPen(0.2, style=QtCore.Qt.DotLine),
                          label='E', labelOpts={'position': 0.97, 'color': 'k'})
        self.plot.addLine(y=0, pen=pg.mkPen(0.2, style=QtCore.Qt.DotLine),
                          label='W', labelOpts={'position': 0.03, 'color': 'k'})

        for r in el_r:
            # noinspection PyUnresolvedReferences
            circle = pg.QtGui.QGraphicsEllipseItem(-r, -r, r * 2, r * 2)
            circle.setPen(pg.mkPen(0.2, style=QtCore.Qt.DotLine))
            self.plot.addItem(circle)

        self.sat_data = self.plot.plot(pen=pg.mkPen(color='r', width=2))
        self.sat_point = self.plot.plot(pen=None, symbol='o', symbolPen=None, symbolSize=6, symbolBrush='r')
        self.ant_data = self.plot.plot(pen=pg.mkPen(color='g', width=2))
        self.ant_point = self.plot.plot(pen=None, symbol='o', symbolPen=None, symbolSize=6, symbolBrush='g')

        vbox_layout = QVBoxLayout()
        vbox_layout.addWidget(self.plot)
        self.setLayout(vbox_layout)

        self.is_tracking = None

    def update_sat_graph(self, azimuth, elevation):
        if self.is_tracking and azimuth and elevation:
            azimuth = np.array(azimuth) - 90.
            elevation = np.abs(np.array(elevation) - 90.)
            x = elevation * np.cos(np.deg2rad(-azimuth))
            y = elevation * np.sin(np.deg2rad(-azimuth))
            self.sat_data.setData(x, y)
            self.sat_point.setData([x[-1]], [y[-1]])

    def update_ant_graph(self, azimuth, elevation):
        if self.is_tracking and azimuth and elevation:
            azimuth = np.array(azimuth) - 90.
            elevation = np.abs(np.array(elevation) - 90.)
            x = elevation * np.cos(np.deg2rad(-azimuth))
            y = elevation * np.sin(np.deg2rad(-azimuth))
            self.ant_data.setData(x, y)
            self.ant_point.setData([x[-1]], [y[-1]])

    def clear_data(self):
        self.sat_data = self.plot.plot(pen=pg.mkPen(color='r', width=2))
        self.ant_data = self.plot.plot(pen=pg.mkPen(color='g', width=2))
