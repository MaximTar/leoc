from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QCloseEvent
from PyQt5.QtWidgets import QMainWindow, QVBoxLayout, QLabel, QPushButton, QWidget, QSplashScreen

from utils.widgets.waiting_spinner_widget import WaitingSpinnerWidget
# from utils.service_worker import ServiceWorker
from threading import Thread


# noinspection PyUnresolvedReferences
class LoadingWindow(QWidget):
    # stop_signal = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__()
        self.setWindowTitle("Waiting...")
        print("LOADING...")
        # self.setWindowModality(Qt.ApplicationModal)
        #
        # # self.showing = False
        # self.cancel = False
        #
        # main_vbox_layout = QVBoxLayout()
        #
        # # self.waiting_spinner = WaitingSpinnerWidget(self)
        # # self.text = QLabel('Service {} is not available, waiting...'.format(client.srv_name))
        # cancel_btn = QPushButton("Cancel")
        # cancel_btn.clicked.connect(self.cancel_btn_clicked)
        #
        # # main_vbox_layout.addWidget(self.waiting_spinner)
        # # main_vbox_layout.addWidget(self.text)
        # main_vbox_layout.addWidget(cancel_btn)
        #
        # # self.worker = ServiceWorker(client)
        # # self.thread = Thread(target=self.worker.do_work)
        # #
        # # # self.stop_signal.connect(self.worker.stop)
        # # # self.worker.moveToThread(self.thread)
        # #
        # # # self.worker.finished.connect(self.thread.quit)
        # # # self.worker.finished.connect(self.worker.deleteLater)
        # # # self.thread.finished.connect(self.thread.deleteLater)
        # # # self.thread.started.connect(self.worker.do_work)
        # # # self.thread.finished.connect(self.worker.stop)
        # #
        # # self.thread.start()
        #
        # # self.client = client
        #
        # widget = QWidget()
        # widget.setLayout(main_vbox_layout)
        # self.setCentralWidget(widget)
        #
        # self.show()
        # # self.run()

    def closeEvent(self, a0: QCloseEvent) -> None:
        print("CLOSE")
        # self.showing = False

    # def show(self) -> None:
    #     super().show()
        # self.showing = True
        # self.waiting_spinner.start()
        # self.run()

    # def cancel_btn_clicked(self):
    #     # self.waiting_spinner.stop()
    #     self.cancel = True
    #     self.close()

    # def set_text(self, text):
    #     self.text.setText(text)

    # def run(self):
    #     thread = QThread()
    #     self.moveToThread(thread)
        # thread = Thread(target=self.show)
        # thread.start()
        # while not self.client.wait_for_service(timeout_sec=1.0):
        #     continue
