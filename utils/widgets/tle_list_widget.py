from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QListWidget, QListWidgetItem
from utils.tle_handler import *


# noinspection PyUnresolvedReferences
class TleListWidget(QListWidget):
    def __init__(self, parent_slot=None):
        super().__init__()
        self.checked_names_list = []

        for name in get_all_names():
            item = QListWidgetItem(name)
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(Qt.Unchecked)
            self.addItem(item)

        self.itemChanged.connect(self.item_changed)
        if parent_slot is not None:
            self.itemChanged.connect(parent_slot)

    def item_changed(self, item):
        if item.checkState() == Qt.Checked:
            self.checked_names_list.append(item.text())
        elif item.checkState() == Qt.Unchecked:
            self.checked_names_list.remove(item.text())

    def update_list(self):
        self.clear()
        for name in get_all_names():
            item = QListWidgetItem(name)
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(Qt.Unchecked)
            self.addItem(item)
