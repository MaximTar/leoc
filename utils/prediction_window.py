from datetime import datetime

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QMainWindow, QTableWidget, QLabel, QTableWidgetItem


class PredictionWindow(QMainWindow):
    def __init__(self, settings=None, orb=None):
        super().__init__()
        self.setWindowTitle("Prediction")
        self.setWindowModality(Qt.ApplicationModal)

        if orb and settings:
            now = datetime.utcnow()
            # noinspection PyBroadException
            try:
                predictions = orb.get_next_passes(now, int(settings.value("general_settings/hours_passes", 24)),
                                                  float(settings.value("general_settings/observer_longitude", 0)),
                                                  float(settings.value("general_settings/observer_latitude", 0)),
                                                  float(settings.value("general_settings/observer_altitude", 0)),
                                                  tol=0.001,
                                                  horizon=int(settings.value("general_settings/horizon_angle", 0)))
                table_widget = QTableWidget(self)
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
        else:
            table_widget = QLabel("No prediction", self)
            table_widget.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        self.setCentralWidget(table_widget)
        self.resize(500, 200)
