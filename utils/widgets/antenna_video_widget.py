import cv2
import time
import threading
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot, Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QHBoxLayout, QLabel, QWidget


# TODO AFTER full screen mode
class AntennaVideoWidget(QWidget):
    class RtspThread(QThread):
        change_pixmap = pyqtSignal(QImage)

        class RtspHandler:
            def __init__(self, rtsp_uri):
                self.rtsp_uri = rtsp_uri
                self.stream = None

            def __enter__(self, *args, **kwargs):
                """ Returns the object which later will have __exit__ called.
                    This relationship creates a context manager. """
                return self

            # noinspection PyShadowingBuiltins
            def __exit__(self, type=None, value=None, traceback=None):
                """ Together with __enter__, allows support for `with-` clauses. """
                self.close()

            def close(self):
                if self.is_opened():
                    self.stream.release()

            def is_opened(self):
                # noinspection PyBroadException
                try:
                    return self.stream is not None and self.stream.isOpened()
                except Exception:
                    return False

        def __init__(self, settings, parent=None, parent_slot=None):
            super().__init__(parent=parent)

            self.settings = settings
            self.handler = self.RtspHandler(self.settings.value("antenna_video/address", "rtsp://10.55.64.20"))
            self.parent_slot = parent_slot

            self._lock = threading.Lock()
            self.running = False

        def run(self):
            self.running = True
            self.handler.close()
            self.handler.stream = cv2.VideoCapture(self.handler.rtsp_uri)
            time.sleep(.5)
            if not self.handler.is_opened() and self.parent_slot:
                self.parent_slot()
            else:
                while self.handler.is_opened() and self.running:
                    with self._lock:
                        ok, rgb_image = self.handler.stream.read()
                        if ok:
                            h, w, ch = rgb_image.shape
                            bytes_per_line = ch * w
                            convert_to_qt_format = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
                            p = convert_to_qt_format.scaled(int(self.settings.value("antenna_video/min_width", 320)),
                                                            int(self.settings.value("antenna_video/min_height", 240)),
                                                            Qt.KeepAspectRatio)
                            self.change_pixmap.emit(p)

        def close_handler(self):
            self.running = False
            with self._lock:
                self.handler.close()

    def __init__(self, settings):
        super().__init__()

        self.label = QLabel(self)
        self.label.setScaledContents(True)

        self.settings = settings

        self.rtsp_thread = None
        self.start_new_rtsp_thread()

        layout_helper = QHBoxLayout()
        layout_helper.addWidget(self.label)

        self.setLayout(layout_helper)

    def start_new_rtsp_thread(self):
        self.rtsp_thread = self.RtspThread(self.settings,
                                           parent=self,
                                           parent_slot=self.if_handler_closed)
        self.rtsp_thread.change_pixmap.connect(self.set_image)
        self.rtsp_thread.start()

    @pyqtSlot(QImage)
    def set_image(self, image):
        self.label.setPixmap(QPixmap.fromImage(image))

    def if_handler_closed(self):
        self.label.setText("Something is wrong\nCheck camera availability")

    def update_thread(self):
        self.rtsp_thread.close_handler()
        self.rtsp_thread.quit()
        self.start_new_rtsp_thread()
