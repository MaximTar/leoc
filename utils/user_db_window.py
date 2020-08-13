from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QMainWindow, QLabel, QVBoxLayout, QWidget, QScrollArea


class UserDbWindow(QMainWindow):
    def __init__(self, string=None, parent=None):
        super().__init__(parent=parent)
        self.setWindowTitle("User Database")
        self.setWindowModality(Qt.ApplicationModal)

        layout = QVBoxLayout()
        scroll_area = QScrollArea()

        lbl = QLabel()
        if string:
            lbl.setText(string)

        scroll_area.setWidget(lbl)
        scroll_area.setMinimumWidth(550)

        layout.addWidget(scroll_area)

        w = QWidget()
        w.setLayout(layout)

        self.setCentralWidget(w)

    def centerize(self, prnt):
        if prnt:
            self.move(
                prnt.window().frameGeometry().topLeft() + prnt.window().rect().center() - self.rect().center())
