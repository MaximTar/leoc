from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QGridLayout, QWidget, QLabel, QSpinBox, QDoubleSpinBox, QMainWindow, QPushButton

from utils.prediction_window import PredictionWindow


# noinspection PyUnresolvedReferences
class PredictionInputWidget(QMainWindow):
    def __init__(self, subs_and_clients, parent=None):
        super(PredictionInputWidget, self).__init__(parent)
        self.parent = parent
        self.setWindowTitle("Prediction")
        self.setWindowModality(Qt.ApplicationModal)

        self.subs_and_clients = subs_and_clients

        start_lbl = QLabel("Start time shifting (hours):")
        length_lbl = QLabel("Number of hours to find passes:")
        horizon_lbl = QLabel("The elevation of horizon:")

        self.start_spinbox = QSpinBox()
        self.length_spinbox = QSpinBox()
        self.horizon_spinbox = QDoubleSpinBox()

        predict_btn = QPushButton("Predict")
        predict_btn.clicked.connect(self.predict_btn_clicked)

        layout = QGridLayout()
        layout.addWidget(start_lbl, 0, 0)
        layout.addWidget(self.start_spinbox, 0, 1)
        layout.addWidget(length_lbl, 1, 0)
        layout.addWidget(self.length_spinbox, 1, 1)
        layout.addWidget(horizon_lbl, 2, 0)
        layout.addWidget(self.horizon_spinbox, 2, 1)
        layout.addWidget(predict_btn, 3, 0, 1, 2)

        central_widget = QWidget()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

    def centerize(self, prnt):
        if prnt:
            self.move(prnt.window().frameGeometry().topLeft() + prnt.window().rect().center() - self.rect().center())

    def predict_btn_clicked(self):
        prediction_window = PredictionWindow(self.subs_and_clients, self.start_spinbox.value(),
                                             self.length_spinbox.value(), self.horizon_spinbox.value(),
                                             parent=self.parent)
        self.close()
        prediction_window.centerize(self.parent)
        prediction_window.show()