from PyQt5.QtWidgets import QWidget, QStackedLayout, QMessageBox

from utils.widgets.satellite_footprint import SatelliteFootprint
from utils.widgets.satellite_track import SatelliteTrack


class MapWidget(QWidget):
    def __init__(self, orb_list=None):
        super().__init__()
        self.orb_list = orb_list

        self.stacked_layout = QStackedLayout()
        self.stacked_layout.setStackingMode(QStackedLayout.StackAll)

        self.background = QWidget()
        self.background.setStyleSheet("border-image: url('resources/earth.jpg') 0 0 0 0 stretch stretch;")
        self.stacked_layout.addWidget(self.background)

        if self.orb_list is not None:
            for orb in self.orb_list:
                track = SatelliteTrack(orb)
                footprint = SatelliteFootprint(orb)
                # TODO warning will drop every time (uncheck after message)
                if track.no_points:
                    QMessageBox.warning(self, "Warning", "No track available", QMessageBox.Ok)
                elif footprint.no_points:
                    QMessageBox.warning(self, "Warning", "No footprint available", QMessageBox.Ok)
                else:
                    self.stacked_layout.addWidget(track)
                    self.stacked_layout.addWidget(footprint)

        self.setLayout(self.stacked_layout)

    def update_map(self, orb_list):
        self.orb_list = orb_list
        for i in reversed(range(self.stacked_layout.count())):
            if self.stacked_layout.itemAt(i).widget() != self.background:
                # noinspection PyTypeChecker
                self.stacked_layout.itemAt(i).widget().setParent(None)
        # not sure, that this is a good solution
        # insert before background
        for orb in orb_list:
            track = SatelliteTrack(orb)
            footprint = SatelliteFootprint(orb)
            if track.no_points:
                QMessageBox.warning(self, "Warning", "No track available", QMessageBox.Ok)
            elif footprint.no_points:
                QMessageBox.warning(self, "Warning", "No footprint available", QMessageBox.Ok)
            else:
                self.stacked_layout.addWidget(track)
                self.stacked_layout.addWidget(footprint)
        # activate widgets
        # for i in reversed(range(self.stacked_layout.count())):
        for i in (range(self.stacked_layout.count())):
            self.stacked_layout.setCurrentIndex(i)
