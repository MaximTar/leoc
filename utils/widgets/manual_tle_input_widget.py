from PyQt5.QtWidgets import QVBoxLayout, QPlainTextEdit, QMainWindow, QWidget, QHBoxLayout, QPushButton, \
    QDialogButtonBox


class ManualTleInputWidget(QMainWindow):
    def __init__(self, parent=None, parent_slot=None):
        super().__init__(parent)
        self.setWindowTitle("Adding TLE to the list manually")

        if parent_slot is not None:
            self.parent_slot = parent_slot

        vbox = QVBoxLayout()
        self.plain_text = QPlainTextEdit()
        self.plain_text.setPlaceholderText("Do not forget about satellite's name in the form: ID NAME\n"
                                           "Example (3le or three-line element format):\n"
                                           "39084 LANDSAT 8\n"
                                           "1 39084U 13008A   20014.41168178  .00000045  00000-0  20099-4 0  9992\n"
                                           "2 39084  98.2026  86.4885 0001270  80.9753 279.1593 14.57115650368125")
        self.plain_text.setUndoRedoEnabled(False)
        vbox.addWidget(self.plain_text)

        button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        vbox.addWidget(button_box)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.cancel)

        central_widget = QWidget()
        central_widget.setLayout(vbox)
        self.setCentralWidget(central_widget)

        self.setMinimumSize(600, 150)
        self.resize(600, 150)
        self.show()

    def cancel(self):
        self.close()

    def accept(self):
        tle_text = self.plain_text.toPlainText()
        tle = tle_text.split('\n')
        self.parent_slot(tle=tle)
        self.close()
