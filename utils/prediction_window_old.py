import datetime

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QBrush, QColor
from PyQt5.QtWidgets import QMainWindow, QTableWidget, QLabel, QTableWidgetItem


class PredictionWindow(QMainWindow):
    def __init__(self, settings=None, orb=None, input_arr=None):
        super().__init__()
        self.setWindowTitle("Prediction")
        self.setWindowModality(Qt.ApplicationModal)

        table_widget = QTableWidget(self)

        if orb and settings:
            now = datetime.datetime.utcnow()
            # noinspection PyBroadException
            try:
                predictions = orb.get_next_passes(now, int(settings.value("general_settings/hours_passes", 24)),
                                                  float(settings.value("general_settings/observer_longitude", 0)),
                                                  float(settings.value("general_settings/observer_latitude", 0)),
                                                  float(settings.value("general_settings/observer_altitude", 0)),
                                                  tol=0.001,
                                                  horizon=int(settings.value("general_settings/horizon_angle", 0)))
                table_widget.setColumnCount(3)
                rows = len(predictions)
                table_widget.setRowCount(rows)
                table_widget.setHorizontalHeaderLabels(["Rise", "Fall", "Max elevation"])
                table_widget.horizontalHeaderItem(0).setToolTip("UTC time")
                table_widget.horizontalHeaderItem(1).setToolTip("UTC time")
                table_widget.horizontalHeaderItem(2).setToolTip("UTC time")
                for i in range(0, rows):
                    prediction = predictions[i]
                    for j in range(0, len(prediction)):
                        table_widget.setItem(i, j, QTableWidgetItem(prediction[j].strftime("%d.%m.%Y %H:%M:%S")))
                table_widget.resizeColumnsToContents()
            except Exception:
                table_widget = QLabel("No prediction", self)
                table_widget.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        elif orb and input_arr:
            now = datetime.datetime.utcnow()
            # noinspection PyBroadException
            try:
                predictions = orb.get_next_passes(utc_time=(now + datetime.timedelta(hours=int(input_arr[0]))),
                                                  length=int(input_arr[1]), lon=37.521575, lat=55.928975, alt=197.,
                                                  tol=0.001, horizon=float(input_arr[2]))
                table_widget.setColumnCount(3)
                rows = len(predictions)
                table_widget.setRowCount(rows)
                table_widget.setHorizontalHeaderLabels(["Rise", "Fall", "Max elevation"])
                table_widget.horizontalHeaderItem(0).setToolTip("UTC time")
                table_widget.horizontalHeaderItem(1).setToolTip("UTC time")
                table_widget.horizontalHeaderItem(2).setToolTip("UTC time")
                for i in range(0, rows):
                    prediction = predictions[i]
                    for j in range(0, len(prediction)):
                        item = QTableWidgetItem(prediction[j].strftime("%d.%m.%Y %H:%M:%S"))
                        item.setFlags(Qt.ItemIsEditable)
                        item.setForeground(QBrush(QColor('black')))
                        table_widget.setItem(i, j, item)
                table_widget.resizeColumnsToContents()
            except Exception:
                table_widget = QLabel("No prediction", self)
                table_widget.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        else:
            table_widget = QLabel("No prediction", self)
            table_widget.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        self.setCentralWidget(table_widget)
        self.resize(500, 200)

    def centerize(self, prnt):
        if prnt:
            self.move(
                prnt.window().frameGeometry().topLeft() + prnt.window().rect().center() - self.rect().center())