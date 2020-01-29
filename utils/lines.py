from PyQt5.QtWidgets import QFrame


# noinspection PyTypeChecker
class VLine(QFrame):
    def __init__(self, flags=None, *args, **kwargs):
        super().__init__(flags, *args, **kwargs)
        self.setFrameShape(self.VLine | self.Sunken)


# noinspection PyTypeChecker
class HLine(QFrame):
    def __init__(self, flags=None, *args, **kwargs):
        super().__init__(flags, *args, **kwargs)
        self.setFrameShape(self.HLine | self.Sunken)
