import sys

from PyQt5.QtCore import QTimer, QSize, Qt
from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import *

from utils.lines import *
from utils.settings_window import SettingsWindow
from utils.tle_handler import *

from utils.widgets.antenna_adjustment_widget import AntennaAdjustmentWidget
from utils.widgets.antenna_control_widget import AntennaControlWidget
from utils.widgets.antenna_graph_widget import AntennaGraphWidget
from utils.widgets.antenna_pose_vel_widget import AntennaPosVelWidget
from utils.widgets.antenna_time_widget import AntennaTimeWidget
from utils.widgets.antenna_video_widget import AntennaVideoWidget
from utils.widgets.manual_tle_input_widget import ManualTleInputWidget
from utils.widgets.map_widget import MapWidget
from utils.widgets.satellite_data_widget import SatelliteDataWidget
from utils.widgets.tle_list_widget import TleListWidget


# import sys  # Suppressing the error messages
#
#
# class DevNull:
#     def write(self, msg):
#         pass
#
#
# sys.stderr = DevNull()  # Suppressing the error messages


# noinspection PyUnresolvedReferences,PyCallByClass,PyArgumentList
class MainWindow(QMainWindow):
    class Status(Enum):
        OK = 0
        NOT_OK = 1

    def __init__(self):
        super().__init__()
        self.setWindowTitle("LEOC")

        main_hbox = QHBoxLayout()
        main_hbox.setContentsMargins(10, 10, 10, 0)

        self.create_status_bar()

        self.settings_window = SettingsWindow()
        self.settings = self.settings_window.settings

        self.map_widget = MapWidget(settings_window=self.settings_window)
        self.tle_list_widget = TleListWidget(self.update_map_widget, data_slot=self.update_sat_data)
        self.satellite_data_widget = SatelliteDataWidget()
        self.antenna_graph_widget = AntennaGraphWidget()
        self.antenna_pose_vel_widget = AntennaPosVelWidget()
        self.antenna_adjustment_widget = AntennaAdjustmentWidget(self.settings)
        self.antenna_control_widget = AntennaControlWidget(self.settings)
        self.antenna_time_widget = AntennaTimeWidget()
        self.antenna_video_widget = AntennaVideoWidget(self.settings)

        self.map_widget.set_uncheck_list_and_slot(self.tle_list_widget.checked_indices_list,
                                                  self.tle_list_widget.uncheck_item)

        # TODO check on combobox changed
        # self.dt = None
        # dt_result = self.antenna_time_widget.get_time_delta()
        # if dt_result == AntennaTimeWidget.TimeResult.OK:
        #     self.dt = self.antenna_time_widget.dt
        #     self.status = self.Status.OK
        # elif dt_result == AntennaTimeWidget.TimeResult.SERVERS_UNAVAILABLE:
        #     QMessageBox.warning(self, "No time received", "NTP servers are unavailable", QMessageBox.Ok)
        #     self.status = self.Status.NOT_OK
        # elif dt_result == AntennaTimeWidget.TimeResult.NO_CONNECTION:
        #     QMessageBox.warning(self, "No time received", "Check your internet connection", QMessageBox.Ok)
        #     self.status = self.Status.NOT_OK

        splitter = QSplitter(Qt.Horizontal)
        splitter.setStretchFactor(1, 1)

        splitter.addWidget(self.construct_left_widget())
        splitter.addWidget(self.map_widget)
        splitter.addWidget(self.construct_right_widget())

        main_hbox.addWidget(splitter)

        # noinspection PyArgumentList
        central_widget = QWidget()
        central_widget.setLayout(main_hbox)
        self.setCentralWidget(central_widget)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_map_widget)
        self.timer.start(int(self.settings.value("general_settings/map_update_period", 1000)))

    def update_map_widget(self):
        tle_list = get_tle_list_by_indices(self.tle_list_widget.checked_indices_list)
        orb_list = get_orb_list_by_tle_list(tle_list)
        self.map_widget.update_map(orb_list)

    def update_sat_data(self):
        tle = get_tle_by_index(self.tle_list_widget.selectedIndexes()[0].row())
        orb = get_orb_by_tle(tle)
        self.satellite_data_widget.update_data(orb)

    def add_to_tle_list_widget(self, sat_id=None, tle=None):
        th = TleHandler()
        if sat_id is not None:
            if is_sat_id(sat_id):
                result = th.save_by_sat_id(sat_id)
            else:
                QMessageBox.warning(self, "No TLE added", "Wrong satellite ID", QMessageBox.Ok)
                return
        elif tle is not None:
            sat_id = tle[0].split(' ').pop(0)
            if is_sat_id(sat_id):
                result = th.save_by_tle(tle)
            else:
                QMessageBox.warning(self, "No TLE added", "Wrong satellite ID", QMessageBox.Ok)
                return
        else:
            QMessageBox.warning(self, "No TLE added", "Something went wrong", QMessageBox.Ok)
            return

        if result == th.Result.IS_NONE:
            QMessageBox.warning(self, "No TLE added", "Something went wrong", QMessageBox.Ok)
        elif result == th.Result.ALREADY_EXISTS:
            sat_name = get_name_by_sat_id(sat_id)
            QMessageBox.information(self, "Find TLE by ID", "{} TLE already in list".format(sat_name), QMessageBox.Ok)
        elif result == th.Result.SAVED:
            self.tle_list_widget.update_list()

    def remove_from_tle_list_widget(self, index):
        remove_tle_by_index(index)
        self.tle_list_widget.update_list()

    def construct_left_widget(self):
        splitter = QSplitter(Qt.Vertical)
        splitter.setStretchFactor(1, 1)
        splitter.addWidget(self.antenna_graph_widget)
        splitter.addWidget(self.antenna_pose_vel_widget)
        splitter.addWidget(self.antenna_adjustment_widget)
        splitter.addWidget(self.antenna_control_widget)
        splitter.addWidget(self.antenna_time_widget)
        splitter.addWidget(self.antenna_video_widget)
        return splitter

    def construct_right_widget(self):
        right_widget = QWidget()
        right_vbox = QVBoxLayout()
        right_vbox.setContentsMargins(5, 10, 10, 0)
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
            # noinspection PyTypeChecker
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
            ret = update_all_tles()
            if ret:
                QMessageBox.information(self, "Something is wrong", ', '.join(ret) + "is not satellite IDs",
                                        QMessageBox.Ok)
        elif update_box.clickedButton() == selected_btn:
            if not self.tle_list_widget.selectedItems():
                QMessageBox.information(self, "Update TLE", "No TLE selected", QMessageBox.Ok)
            else:
                ret = update_tle_by_index(self.tle_list_widget.selectionModel().selectedIndexes()[0].row())
                if ret:
                    QMessageBox.information(self, "Something is wrong", str(ret) + "is not satellite ID",
                                            QMessageBox.Ok)

    def create_status_bar(self):
        # self.statusBar().showMessage("")
        status_lbl = QLabel("Status: ")

        settings_btn = QPushButton("", self)
        settings_btn.setIcon(QIcon("resources/icons/settings.png"))
        settings_btn.setIconSize(QSize(20, 20))
        settings_btn.clicked.connect(self.show_settings_window)

        status_btn = QPushButton("", self)
        status_btn.setIcon(QIcon("resources/icons/status_ok.png"))
        status_btn.setIconSize(QSize(20, 20))

        self.statusBar().setStyleSheet("QStatusBar {border: 0; background-color: #FFF8DC;}")

        self.statusBar().addWidget(settings_btn)
        self.statusBar().addPermanentWidget(VLine())
        self.statusBar().addPermanentWidget(status_lbl)
        self.statusBar().addPermanentWidget(status_btn)
        self.statusBar().addPermanentWidget(VLine())

    def show_settings_window(self):
        self.settings_window.show()

    def send_btn_clicked(self):
        # TODO INTERACTION
        pass

    def update_settings(self):
        # TODO
        pass


if __name__ == "__main__":
    app = QApplication(sys.argv)

    main = MainWindow()
    main.show()
    sys.exit(app.exec_())
