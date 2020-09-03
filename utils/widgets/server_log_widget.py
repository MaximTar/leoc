from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QWidget, QTextEdit, QVBoxLayout, QHBoxLayout, QRadioButton


# noinspection PyUnresolvedReferences
class ServerLogWidget(QWidget):
    update_signal = pyqtSignal()

    def __init__(self):
        super().__init__()

        vbl = QVBoxLayout()

        hbl = QHBoxLayout()
        self.info_btn = QRadioButton("INFO")
        self.info_btn.setChecked(True)
        self.info_btn.toggled.connect(lambda: self.btn_state(self.info_btn))
        hbl.addWidget(self.info_btn)

        self.warn_btn = QRadioButton("WARNING")
        self.warn_btn.toggled.connect(lambda: self.btn_state(self.warn_btn))
        hbl.addWidget(self.warn_btn)

        self.err_btn = QRadioButton("ERROR")
        self.err_btn.toggled.connect(lambda: self.btn_state(self.err_btn))
        hbl.addWidget(self.err_btn)

        self.cri_btn = QRadioButton("CRITICAL")
        self.cri_btn.toggled.connect(lambda: self.btn_state(self.cri_btn))
        hbl.addWidget(self.cri_btn)

        vbl.addLayout(hbl)

        self.text_edit = QTextEdit()
        self.text_edit.setReadOnly(True)
        vbl.addWidget(self.text_edit)

        self.setLayout(vbl)

        self.msgs = []
        self.text = ""
        self.lvl = 20
        self.update_signal.connect(self.update_text_edit)

    def btn_state(self, b):
        if b.text() == "INFO" and b.isChecked():
            self.lvl = 20
        elif b.text() == "WARNING" and b.isChecked():
            self.lvl = 30
        elif b.text() == "ERROR" and b.isChecked():
            self.lvl = 40
        elif b.text() == "CRITICAL" and b.isChecked():
            self.lvl = 50
        else:
            self.lvl = 20

    def update_text(self, stamp_s, stamp_ns, lvl, msg):
        if lvl >= self.lvl:
            self.msgs.reverse()
            if len(self.msgs) < 20:
                self.msgs.append(msg)
            else:
                self.msgs.pop(0)
                self.msgs.append(msg)
                self.msgs.reverse()

            self.text = '\n'.join(self.msgs)
            self.update_signal.emit()

    def update_text_edit(self):
        self.text_edit.setPlainText(self.text)
