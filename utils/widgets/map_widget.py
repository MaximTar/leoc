import os

from PyQt5.QtCore import QEvent, Qt, QPointF
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QWidget, QStackedLayout, QMessageBox, QAction, QMenu, QLabel

from utils.qpoints_utils import qpoints_scaling
from utils.widgets.satellite_footprint_widget import SatelliteFootprintWidget
from utils.widgets.satellite_track_widget import SatelliteTrackWidget
from utils.widgets.terminator_widget import TerminatorWidget

MCC_ICON_PATH = os.path.dirname(os.path.abspath(__file__)) + "/../../resources/icons/mcc.png"
MCC_ICON_SIZE = 30


class MapWidget(QWidget):
    # not sure, that this is a good solution (to pass slot, list and window to the constructor)
    def __init__(self, orb_list=None, uncheck_slot=None, idx_list=None, settings_window=None):
        super().__init__()
        self.uncheck_slot = uncheck_slot
        self.idx_list = idx_list
        self.settings_window = settings_window

        self.orb_list = orb_list

        self.stacked_layout = QStackedLayout()
        self.stacked_layout.setStackingMode(QStackedLayout.StackAll)

        self.background = QWidget()
        self.background.setStyleSheet("border-image: url('resources/earth.jpg') 0 0 0 0 stretch stretch;")
        self.stacked_layout.addWidget(self.background)

        self.mcc = QLabel("", self)
        self.mcc.setPixmap(
            QPixmap(MCC_ICON_PATH).scaled(MCC_ICON_SIZE, MCC_ICON_SIZE, Qt.KeepAspectRatio, Qt.FastTransformation))
        # self.mcc.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        # mcc_coordinates = [float(self.settings_window.settings.value("general_settings/observer_longitude", 0)),
        #                    float(self.settings_window.settings.value("general_settings/observer_latitude", 0)),
        #                    float(self.settings_window.settings.value("general_settings/observer_altitude", 0))]
        self.mcc_qpoint = [
            QPointF(float(self.settings_window.settings.value("general_settings/observer_longitude", 0)) + 180,
                    -float(self.settings_window.settings.value("general_settings/observer_latitude", 0)) + 90)]
        scaled_mcc_qpoint = qpoints_scaling(self.width(), self.height(), self.mcc_qpoint)[0]
        print(self.mcc.geometry())
        self.mcc.setGeometry(scaled_mcc_qpoint.x() - MCC_ICON_SIZE / 2, scaled_mcc_qpoint.y() - MCC_ICON_SIZE / 2,
                             MCC_ICON_SIZE, MCC_ICON_SIZE)
        print(self.mcc.geometry())
        # self.mcc.setGeometry(0, 0, 0, 0)

        self.stacked_layout.addWidget(self.mcc)

        if self.orb_list is not None:
            for orb in self.orb_list:
                track = SatelliteTrackWidget(orb)
                footprint = SatelliteFootprintWidget(orb)
                if track.no_points:
                    QMessageBox.warning(self, "Warning", "No track available", QMessageBox.Ok)
                    if self.uncheck_slot and self.idx_list:
                        idx = orb_list.index(orb)
                        self.uncheck_slot(self.idx_list[idx])
                elif footprint.no_points:
                    QMessageBox.warning(self, "Warning", "No footprint available", QMessageBox.Ok)
                    if self.uncheck_slot and self.idx_list:
                        idx = orb_list.index(orb)
                        self.uncheck_slot(self.idx_list[idx])
                else:
                    self.stacked_layout.addWidget(track)
                    self.stacked_layout.addWidget(footprint)

        terminator = TerminatorWidget()
        self.stacked_layout.addWidget(terminator)

        self.setLayout(self.stacked_layout)

        # context menu
        self.installEventFilter(self)

    def paintEvent(self, event):
        scaled_mcc_qpoint = qpoints_scaling(self.width(), self.height(), self.mcc_qpoint)[0]
        self.mcc.setGeometry(scaled_mcc_qpoint.x() - MCC_ICON_SIZE / 2, scaled_mcc_qpoint.y() - MCC_ICON_SIZE / 2,
                             MCC_ICON_SIZE, MCC_ICON_SIZE)

    def update_map(self, orb_list):
        self.orb_list = orb_list
        for i in reversed(range(self.stacked_layout.count())):
            if self.stacked_layout.itemAt(i).widget() != self.background \
                    and self.stacked_layout.itemAt(i).widget() != self.mcc:
                # noinspection PyTypeChecker
                self.stacked_layout.itemAt(i).widget().setParent(None)
        # not sure, that this is a good solution
        # insert before background
        for orb in orb_list:
            track = SatelliteTrackWidget(orb)
            footprint = SatelliteFootprintWidget(orb)
            if track.no_points:
                QMessageBox.warning(self, "Warning", "No track available", QMessageBox.Ok)
                if self.uncheck_slot and self.idx_list:
                    idx = orb_list.index(orb)
                    self.uncheck_slot(self.idx_list[idx])
            elif footprint.no_points:
                QMessageBox.warning(self, "Warning", "No footprint available", QMessageBox.Ok)
                if self.uncheck_slot and self.idx_list:
                    idx = orb_list.index(orb)
                    self.uncheck_slot(self.idx_list[idx])
            else:
                self.stacked_layout.addWidget(track)
                self.stacked_layout.addWidget(footprint)

        terminator = TerminatorWidget()
        self.stacked_layout.addWidget(terminator)

        # activate widgets
        # for i in reversed(range(self.stacked_layout.count())):
        for i in (range(self.stacked_layout.count())):
            self.stacked_layout.setCurrentIndex(i)

    def set_uncheck_list_and_slot(self, idx_list, slot):
        if slot is not None and idx_list is not None:
            self.idx_list = idx_list
            self.uncheck_slot = slot

    def eventFilter(self, obj, event):
        if event.type() == QEvent.ContextMenu:
            set_observer_position_action = QAction(self.tr("Set observer position"), self)
            set_observer_position_action.triggered.connect(self.set_observer_position_slot)

            menu = QMenu(self)
            menu.addAction(set_observer_position_action)
            menu.exec_(event.globalPos())
            return True
        return False

    def set_observer_position_slot(self):
        if self.settings_window is not None:
            self.settings_window.tabs.setCurrentIndex(0)
            self.settings_window.show()
