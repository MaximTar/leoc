from PyQt5.QtWidgets import QWidget, QVBoxLayout
from pyqtgraph.Qt import QtCore
import pyqtgraph as pg
import numpy as np


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
                          label='180', labelOpts={'position': 0.97, 'color': 'k'})
        self.plot.addLine(x=0, pen=pg.mkPen(0.2, style=QtCore.Qt.DotLine),
                          label='0', labelOpts={'position': 0.03, 'color': 'k'})
        self.plot.addLine(y=0, pen=pg.mkPen(0.2, style=QtCore.Qt.DotLine),
                          label='90', labelOpts={'position': 0.97, 'color': 'k'})
        self.plot.addLine(y=0, pen=pg.mkPen(0.2, style=QtCore.Qt.DotLine),
                          label='270', labelOpts={'position': 0.04, 'color': 'k'})

        for r in self.el_r:
            # noinspection PyUnresolvedReferences
            circle = pg.QtGui.QGraphicsEllipseItem(-r, -r, r * 2, r * 2)
            circle.setPen(pg.mkPen(0.2, style=QtCore.Qt.DotLine))
            self.plot.addItem(circle)

        self.data = self.plot.plot()

        vbox_layout = QVBoxLayout()
        vbox_layout.addWidget(self.plot)
        self.setLayout(vbox_layout)

    def update_graph(self, azimuth, elevation):
        # azimuth and elevation should be in radians
        x = elevation * np.cos(azimuth)
        y = elevation * np.sin(azimuth)
        self.data.setData(x, y)
