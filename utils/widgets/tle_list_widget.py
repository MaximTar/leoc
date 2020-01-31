from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QListWidget, QListWidgetItem
from utils.tle_handler import *


# noinspection PyUnresolvedReferences
class TleListWidget(QListWidget):
    def __init__(self, parent_slot=None, data_slot=None):
        super().__init__()
        self.checked_indices_list = []

        for name in get_all_names():
            item = QListWidgetItem(name)
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(Qt.Unchecked)
            self.addItem(item)

        if data_slot:
            self.itemSelectionChanged.connect(data_slot)

        self.itemChanged.connect(self.item_changed)
        if parent_slot and self.checked_indices_list:
            self.itemChanged.connect(parent_slot)

    def item_changed(self, item):
        if item.checkState() == Qt.Checked:
            self.checked_indices_list.append(self.indexFromItem(item).row())
        elif item.checkState() == Qt.Unchecked:
            self.checked_indices_list.remove(self.indexFromItem(item).row())

    def update_list(self):
        self.clear()
        # TODO AFTER think about how to save checked and selected items
        for name in get_all_names():
            item = QListWidgetItem(name)
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(Qt.Unchecked)
            self.addItem(item)

    def uncheck_item(self, item_idx):
        self.item(item_idx).setCheckState(Qt.Unchecked)

