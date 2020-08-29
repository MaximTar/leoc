from PyQt5.QtCore import QObject, pyqtSignal, QThread
from utils.loading_window import LoadingWindow


# noinspection PyUnresolvedReferences
class ServiceWorker(QObject):
    finished = pyqtSignal()

    def __init__(self, client, parent=None):
        QObject.__init__(self, parent=parent)
        self.continue_run = True
        self.client = client

        lw = LoadingWindow(client)
        thread = QThread()
        lw.moveToThread(thread)
        thread.start()

    def do_work(self):
        print(self.client)
        while not self.client.wait_for_service(timeout_sec=1.0):
            print('here2')
            # if not self.continue_run:
            #     break
            continue
        # self.finished.emit()

    # def stop(self):
    #     self.continue_run = False
