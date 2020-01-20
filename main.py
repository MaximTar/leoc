import sys

from PyQt5.QtWidgets import *

from utils.widgets.manual_tle_input_widget import ManualTleInputWidget
from utils.widgets.map_widget import MapWidget
from utils.widgets.tle_list_widget import *
from utils.satellite.satellite_footprint import *


class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()

        main_hbox = QHBoxLayout()

        self.map_widget = MapWidget()
        self.tle_list_widget = TleListWidget(self.update_map_widget)

        splitter = QSplitter(Qt.Horizontal)
        splitter.setStretchFactor(1, 1)

        splitter.addWidget(self.map_widget)
        splitter.addWidget(self.construct_right_widget())

        main_hbox.addWidget(splitter)

        # noinspection PyArgumentList
        central_widget = QWidget()
        central_widget.setLayout(main_hbox)
        self.setCentralWidget(central_widget)

    def update_map_widget(self):
        tle_list = get_tle_list_by_names(self.tle_list_widget.checked_names_list)
        orb_list = get_orb_list_by_tle_list(tle_list)
        self.map_widget.update_map(orb_list)

    def add_to_tle_list_widget(self, sat_id=None, tle=None):
        th = TleHandler()
        if sat_id is not None:
            result = th.save_by_sat_id(sat_id)
        else:
            result = th.save_by_tle(tle)

        if result == th.Result.IS_NONE:
            # TODO
            pass
        elif result == th.Result.ALREADY_EXISTS:
            # TODO return sat_id to the user / get sat name
            QMessageBox.information(self, "Find TLE by ID", "TLE already in list", QMessageBox.Ok)
        elif result == th.Result.SAVED:
            self.tle_list_widget.update_list()

    def remove_from_tle_list_widget(self, index):
        remove_tle_by_index(index)
        self.tle_list_widget.update_list()

    def construct_right_widget(self):
        right_widget = QWidget()
        right_vbox = QVBoxLayout()
        btn_widget = QWidget()
        btn_box = QHBoxLayout()

        add_btn = QPushButton("+", self)
        add_btn.setToolTip("Add TLE to the list")
        add_btn.clicked.connect(self.add_btn_clicked)

        remove_btn = QPushButton("-", self)
        remove_btn.setToolTip("Remove TLE from the list")
        remove_btn.clicked.connect(self.remove_btn_clicked)

        update_btn = QPushButton("Update", self)
        update_btn.setToolTip("Update")
        update_btn.clicked.connect(self.update_btn_clicked)

        send_btn = QPushButton("Send", self)
        send_btn.setToolTip("Send TLE")
        update_btn.clicked.connect(self.send_btn_clicked)

        btn_box.addWidget(add_btn)
        btn_box.addWidget(remove_btn)
        btn_box.addWidget(update_btn)
        btn_box.addWidget(send_btn)
        btn_widget.setLayout(btn_box)

        right_vbox.addWidget(self.tle_list_widget)
        right_vbox.addWidget(btn_widget)
        right_widget.setLayout(right_vbox)
        return right_widget

    def add_btn_clicked(self):
        add_box = QMessageBox(self)
        add_box.setStandardButtons(QMessageBox.Cancel)
        find_btn = add_box.addButton("Find by ID (need internet)", QMessageBox.ActionRole)
        manual_btn = add_box.addButton("Add manually", QMessageBox.ActionRole)
        add_box.setDefaultButton(QMessageBox.Cancel)

        add_box.exec()

        if add_box.clickedButton() == find_btn:
            sat_id, ok = QInputDialog.getInt(self, "Find TLE by satellite ID", "Satellite ID", min=0)
            if ok:
                self.add_to_tle_list_widget(sat_id=sat_id)
        elif add_box.clickedButton() == manual_btn:
            ManualTleInputWidget(parent=self, parent_slot=self.add_to_tle_list_widget)

    def remove_btn_clicked(self):
        remove_box = QMessageBox(self)
        selected_items = self.tle_list_widget.selectedItems()
        if not selected_items:
            remove_box.setText("No TLE selected")
            remove_box.setStandardButtons(QMessageBox.Ok)
            remove_box.exec()
        else:
            remove_box.setText("Are you really want to remove {} from the list?"
                               .format(selected_items[0].text()))
            remove_box.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
            result = remove_box.exec()
            if result == QMessageBox.Yes:
                self.remove_from_tle_list_widget(self.tle_list_widget.selectionModel().selectedIndexes()[0].row())

    def update_btn_clicked(self):
        update_box = QMessageBox(self)
        update_box.setText("Do you want to update all TLEs or only selected one?")
        update_box.setStandardButtons(QMessageBox.Cancel)
        all_btn = update_box.addButton("All", QMessageBox.ActionRole)
        selected_btn = update_box.addButton("Selected", QMessageBox.ActionRole)

        update_box.exec()

        if update_box.clickedButton() == all_btn:
            update_all_tles()
        elif update_box.clickedButton() == selected_btn:
            if not self.tle_list_widget.selectedItems():
                QMessageBox.information(self, "Update TLE", "No TLE selected", QMessageBox.Ok)
            else:
                update_tle_by_index(self.tle_list_widget.selectionModel().selectedIndexes()[0].row())

    def send_btn_clicked(self):
        # TODO
        pass


if __name__ == "__main__":
    app = QApplication(sys.argv)

    main = MainWindow()
    main.show()
    sys.exit(app.exec_())
