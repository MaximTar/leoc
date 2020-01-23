import cv2
import time
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot, Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QHBoxLayout, QLabel, QWidget


# TODO full screen mode
class AntennaVideoWidget(QWidget):
    class RtspThread(QThread):
        change_pixmap = pyqtSignal(QImage)

        # TODO think about moving methods from handler to the thread class
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

        def __init__(self, rtsp_uri, parent=None, parent_slot=None):
            super().__init__(parent=parent)
            self.handler = self.RtspHandler(rtsp_uri)
            self.parent_slot = parent_slot

        def run(self):
            self.handler.close()
            self.handler.stream = cv2.VideoCapture(self.handler.rtsp_uri)
            time.sleep(.5)
            if not self.handler.is_opened() and self.parent_slot is not None:
                self.parent_slot()
            while self.handler.is_opened():
                ok, rgb_image = self.handler.stream.read()
                if ok:
                    h, w, ch = rgb_image.shape
                    bytes_per_line = ch * w
                    convert_to_qt_format = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
                    # TODO settings
                    p = convert_to_qt_format.scaled(320, 240, Qt.KeepAspectRatio)
                    self.change_pixmap.emit(p)

    def __init__(self, rtsp_uri):
        super().__init__()

        self.label = QLabel(self)
        self.label.setScaledContents(True)
        th = self.RtspThread(rtsp_uri, parent=self, parent_slot=self.if_handler_closed)
        th.change_pixmap.connect(self.set_image)
        th.start()

        layout_helper = QHBoxLayout()
        layout_helper.addWidget(self.label)

        self.setLayout(layout_helper)

    @pyqtSlot(QImage)
    def set_image(self, image):
        self.label.setPixmap(QPixmap.fromImage(image))

    def if_handler_closed(self):
        self.label.setText("Something is wrong\nCheck camera availability")
