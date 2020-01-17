from PyQt5.QtWidgets import QWidget, QStackedLayout

from utils.satellite.satellite_footprint import SatelliteFootprint
from utils.satellite.satellite_track import SatelliteTrack


class MapWidget(QWidget):
    def __init__(self, orb_list=None):
        super().__init__()

        self.stacked_layout = QStackedLayout()
        self.stacked_layout.setStackingMode(QStackedLayout.StackAll)

        self.background = QWidget()
        self.background.setStyleSheet("border-image: url('resources/earth.jpg') 0 0 0 0 stretch stretch;")
        self.stacked_layout.addWidget(self.background)

        if orb_list is not None:
            for orb in orb_list:
                self.stacked_layout.addWidget(SatelliteTrack(orb))
                self.stacked_layout.addWidget(SatelliteFootprint(orb))

        self.setLayout(self.stacked_layout)

    def update_map(self, orb_list):
        for i in reversed(range(self.stacked_layout.count())):
            if self.stacked_layout.itemAt(i).widget() != self.background:
                # noinspection PyTypeChecker
                self.stacked_layout.itemAt(i).widget().setParent(None)
        # not sure, that this is a good solution
        # insert before background
        for orb in orb_list:
            self.stacked_layout.insertWidget(0, SatelliteTrack(orb))
            self.stacked_layout.insertWidget(0, SatelliteFootprint(orb))
        # activate widgets
        for i in reversed(range(self.stacked_layout.count())):
            self.stacked_layout.setCurrentIndex(i)
