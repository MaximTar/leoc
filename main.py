import sys
from threading import Thread

from PyQt5.QtCore import QTimer, QSize, Qt
from PyQt5.QtGui import QIcon, QColor
from PyQt5.QtWidgets import *
from antenna_interfaces.srv import *

from utils.lines import *
from utils.prediction_window import PredictionWindow
from utils.settings_window import SettingsWindow
from utils.subs_and_clients import SubscribersAndClients
from utils.tle_handler import *
from utils.widgets.antenna_adjustment_widget import AntennaAdjustmentWidget
from utils.widgets.antenna_control_widget import AntennaControlWidget
from utils.widgets.antenna_graph_widget import AntennaGraphWidget
from utils.widgets.antenna_pose_widget import AntennaPoseWidget
from utils.widgets.antenna_time_widget import AntennaTimeWidget
from utils.widgets.antenna_video_widget import AntennaVideoWidget
from utils.widgets.login_widget import LoginWidget
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

# TODO AFTER-AFTER Time simulation mode
# noinspection PyUnresolvedReferences,PyCallByClass,PyArgumentList
class MainWindow(QMainWindow):
    class Status(Enum):
        OK = 1
        NOT_OK = 0

    def __init__(self):
        super().__init__()
        self.setWindowTitle("LEOC")
        # TODO MAKE FUNCTIONAL TO RELOGIN

        main_hbox = QHBoxLayout()
        main_hbox.setContentsMargins(10, 10, 10, 0)

        self.set_active_btn = QPushButton("Set active", self)
        self.set_active_btn.setToolTip("Set active satellite")
        self.set_active_btn.clicked.connect(self.set_active_btn_clicked)
        self.set_active_btn.setEnabled(False)

        self.dt_status = self.Status.NOT_OK

        self.create_status_bar()

        self.settings_window = SettingsWindow(parent_upd_slot=self.update_settings)
        self.prediction_window = PredictionWindow()
        self.settings = self.settings_window.settings

        self.map_widget = MapWidget(settings_window=self.settings_window)
        self.tle_list_widget = TleListWidget(subs_and_clients, parent_slot=self.update_map_widget,
                                             data_slot=self.update_sat_data)
        self.satellite_data_widget = SatelliteDataWidget(settings=self.settings)
        self.antenna_graph_widget = AntennaGraphWidget()
        self.antenna_pose_widget = AntennaPoseWidget()
        self.antenna_adjustment_widget = AntennaAdjustmentWidget(self.settings)
        self.antenna_control_widget = AntennaControlWidget()
        self.antenna_time_widget = AntennaTimeWidget(dt_slot=self.set_dt)
        self.antenna_video_widget = AntennaVideoWidget(self.settings)

        self.map_widget.set_uncheck_list_and_slot(self.tle_list_widget.checked_indices_list,
                                                  self.tle_list_widget.uncheck_item)

        get_tles(self.tle_list_widget, subs_and_clients)

        self.dt = None
        # dt_result = self.antenna_time_widget.get_time_delta()
        # if dt_result == AntennaTimeWidget.TimeResult.OK:
        #     self.set_dt(self.antenna_time_widget.dt)
        #     self.dt_status = self.Status.OK
        # elif dt_result == AntennaTimeWidget.TimeResult.SERVERS_UNAVAILABLE:
        #     QMessageBox.warning(self, "No time received", "NTP servers are unavailable", QMessageBox.Ok)
        #     self.dt_status = self.Status.NOT_OK
        # elif dt_result == AntennaTimeWidget.TimeResult.NO_CONNECTION:
        #     QMessageBox.warning(self, "No time received", "Check your internet connection", QMessageBox.Ok)
        #     self.dt_status = self.Status.NOT_OK

        splitter = QSplitter(Qt.Horizontal)

        lw = self.construct_left_widget()
        lw.setMinimumSize(250, 400)
        splitter.addWidget(lw)
        mw = self.map_widget
        splitter.addWidget(mw)
        rw = self.construct_right_widget()
        rw.setMinimumSize(250, 400)
        splitter.addWidget(rw)

        splitter.setSizes([0, 640, 0])
        main_hbox.addWidget(splitter)

        # noinspection PyArgumentList
        central_widget = QWidget()
        central_widget.setLayout(main_hbox)
        self.setCentralWidget(central_widget)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_map_widget)
        self.timer.timeout.connect(self.update_sat_data)
        self.timer.start(int(self.settings.value("general_settings/map_update_period", 1000)))

        self.background_color = None
        self.rid = None
        self.check_active_satellite()
        subs_and_clients.set_sat_graph_slot(self.antenna_graph_widget.update_graph)
        subs_and_clients.set_ant_pose_slot(self.antenna_pose_widget.update_pose)

        self.login_widget = LoginWidget(subs_and_clients)
        self.login_widget.show()

    #     self.timer2 = QTimer()
    #     self.timer2.timeout.connect(self.test)
    #     self.timer2.start(3000)
    #     self.flag = True
    #
    # def test(self):
    #     if self.flag:
    #         self.antenna_video_widget.update_thread()
    #         self.flag = False

    def update_timer(self):
        self.timer.start(int(self.settings.value("general_settings/map_update_period", 1000)))

    def set_dt(self, dt):
        if dt:
            self.dt = dt
            self.dt_status = self.Status.OK
            self.update_set_btn_state()

    def update_map_widget(self):
        tle_list = get_tle_list_by_indices(self.tle_list_widget.checked_indices_list)
        orb_list = get_orb_list_by_tle_list(tle_list)
        self.map_widget.update_map(orb_list)

    def update_sat_data(self):
        if self.tle_list_widget.selectedIndexes():
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
        splitter.addWidget(self.antenna_pose_widget)
        splitter.addWidget(self.antenna_adjustment_widget)
        splitter.addWidget(self.antenna_control_widget)
        # splitter.addWidget(self.antenna_time_widget)
        splitter.addWidget(self.antenna_video_widget)
        return splitter

    def construct_right_widget(self):
        right_widget = QWidget()
        right_vbox = QVBoxLayout()
        right_vbox.setContentsMargins(5, 10, 10, 0)
        btn_widget = QWidget()
        btn_grid = QGridLayout()

        add_btn = QPushButton("+", self)
        add_btn.setToolTip("Add TLE to the list")
        add_btn.clicked.connect(self.add_btn_clicked)

        remove_btn = QPushButton("-", self)
        remove_btn.setToolTip("Remove TLE from the list")
        remove_btn.clicked.connect(self.remove_btn_clicked)

        update_btn = QPushButton("Update", self)
        update_btn.setToolTip("Update")
        update_btn.clicked.connect(self.update_btn_clicked)

        predict_btn = QPushButton("Predict", self)
        predict_btn.setToolTip("Make prediction")
        predict_btn.clicked.connect(self.predict_btn_clicked)

        btn_grid.addWidget(predict_btn, 0, 0, 1, 10)
        btn_grid.addWidget(self.set_active_btn, 0, 10, 1, 10)
        btn_grid.addWidget(add_btn, 2, 0, 1, 6)
        btn_grid.addWidget(remove_btn, 2, 7, 1, 6)
        btn_grid.addWidget(update_btn, 2, 14, 1, 6)

        btn_widget.setLayout(btn_grid)

        splitter = QSplitter(Qt.Vertical)
        splitter.addWidget(self.tle_list_widget)
        scroll_area = QScrollArea()
        scroll_area.setWidget(self.satellite_data_widget)
        splitter.addWidget(scroll_area)

        splitter.setStretchFactor(0, 9)
        splitter.setStretchFactor(1, 1)

        right_vbox.addWidget(splitter)
        right_vbox.addWidget(btn_widget)
        right_widget.setLayout(right_vbox)
        return right_widget

    def update_set_btn_state(self):
        self.set_active_btn.setEnabled(bool(self.dt_status.value))

    def add_btn_clicked(self):
        add_box = QMessageBox(self)
        add_box.setStandardButtons(QMessageBox.Cancel)
        find_btn = add_box.addButton("By ID", QMessageBox.ActionRole)
        manual_btn = add_box.addButton("Manually", QMessageBox.ActionRole)
        add_box.setDefaultButton(QMessageBox.Cancel)

        add_box.exec()

        if add_box.clickedButton() == find_btn:
            sat_id, ok = QInputDialog.getInt(self, "Find TLE by satellite ID", "Satellite ID", min=0)
            if ok:
                req = SatsAdd.Request()
                req.num = str(sat_id)
                while not subs_and_clients.sat_add_client.wait_for_service(timeout_sec=1.0):
                    # TODO LOADING
                    print('Service sat_add_client is not available, waiting...')

                future = subs_and_clients.sat_add_client.call_async(req)
                while rclpy.ok():
                    # TODO LOADING
                    if future.done():
                        try:
                            response = future.result()
                        except Exception as e:
                            QMessageBox.warning(self, "sat_add_client", "Cannot add satellite to list.\n"
                                                                        "Stacktrace: {}".format(e), QMessageBox.Ok)
                        else:
                            if response.res:
                                self.tle_list_widget.update_list()
                            else:
                                QMessageBox.warning(self, "sat_add_client",
                                                    "Cannot add satellite to list.",
                                                    QMessageBox.Ok)
                        break
        elif add_box.clickedButton() == manual_btn:
            req = TlesUserSet.Request()
            fname = QFileDialog.getOpenFileName(self, 'Open file', "Text files (*.txt)")[0]
            f = open(fname, 'r')
            with f:
                data = f.read()
            f.close()
            req.tles = data

            while not subs_and_clients.usr_tle_set_client.wait_for_service(timeout_sec=1.0):
                # TODO LOADING
                print('Service sat_add_client is not available, waiting...')

            future = subs_and_clients.usr_tle_set_client.call_async(req)
            while rclpy.ok():
                # TODO LOADING
                if future.done():
                    try:
                        response = future.result()
                    except Exception as e:
                        QMessageBox.warning(self, "sat_add_client", "Cannot add satellite to list.\n"
                                                                    "Stacktrace: {}".format(e), QMessageBox.Ok)
                    else:
                        if response.res:
                            QMessageBox.information(self, "Done", "TLEs added to user database", QMessageBox.Ok)
                        else:
                            QMessageBox.warning(self, "sat_add_client", "Cannot add satellite to list.", QMessageBox.Ok)
                    break
            # noinspection PyTypeChecker
            # ManualTleInputWidget(parent=self, parent_slot=self.add_to_tle_list_widget)

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
                req = SatsDel.Request()
                req.id = int(selected_items[0].statusTip())
                while not subs_and_clients.sat_del_client.wait_for_service(timeout_sec=1.0):
                    # TODO LOADING
                    print('Service sat_del_client is not available, waiting...')

                future = subs_and_clients.sat_del_client.call_async(req)
                while rclpy.ok():
                    # TODO LOADING
                    if future.done():
                        try:
                            response = future.result()
                        except Exception as e:
                            QMessageBox.warning(self, "sat_del_client", "Cannot delete satellite from list.\n"
                                                                        "Stacktrace: {}".format(e), QMessageBox.Ok)
                        else:
                            if response.res:
                                self.tle_list_widget.update_list()
                            else:
                                QMessageBox.warning(self, "sat_del_client", "Cannot delete satellite from list.",
                                                    QMessageBox.Ok)
                        break

    def update_btn_clicked(self):
        update_box = QMessageBox(self)
        update_box.setText("Do you want to update all TLEs or only selected one?")
        update_box.setStandardButtons(QMessageBox.Cancel)
        all_btn = update_box.addButton("All", QMessageBox.ActionRole)
        selected_btn = update_box.addButton("Selected", QMessageBox.ActionRole)

        update_box.exec()

        if update_box.clickedButton() == all_btn:
            req = SatsUpdate.Request()
            while not subs_and_clients.sat_upd_client.wait_for_service(timeout_sec=1.0):
                # TODO LOADING
                print('Service sat_upd_client is not available, waiting...')

            future = subs_and_clients.sat_upd_client.call_async(req)
            while rclpy.ok():
                # TODO LOADING
                if future.done():
                    try:
                        response = future.result()
                    except Exception as e:
                        QMessageBox.warning(self, "sat_upd_client", "Cannot update satellites list.\n"
                                                                    "Stacktrace: {}".format(e), QMessageBox.Ok)
                    else:
                        # TODO if-else & MSG_BOX (get satellites by id and show names to user)
                        subs_and_clients.get_logger().info(
                            'Result: %s' % response)
                    break

        elif update_box.clickedButton() == selected_btn:
            if not self.tle_list_widget.selectedItems():
                QMessageBox.information(self, "Update TLE", "No TLE selected", QMessageBox.Ok)
            else:
                ret = update_tle_by_index(self.tle_list_widget.selectionModel().selectedIndexes()[0].row())
                if ret:
                    QMessageBox.information(self, "Something is wrong", str(ret) + "is not satellite ID",
                                            QMessageBox.Ok)

    def predict_btn_clicked(self):
        if self.tle_list_widget.selectedIndexes():
            tle = get_tle_by_index(self.tle_list_widget.selectedIndexes()[0].row())
            orb = get_orb_by_tle(tle)
            self.prediction_window = PredictionWindow(settings=self.settings, orb=orb)
            self.prediction_window.show()
        else:
            QMessageBox.information(self, "Prediction", "Select satellite first!", QMessageBox.Ok)

    def create_status_bar(self):
        # self.statusBar().showMessage("")
        status_lbl = QLabel("Status: ")

        settings_btn = QPushButton("", self)
        settings_btn.setIcon(QIcon(os.path.dirname(os.path.abspath(__file__)) + "/resources/icons/settings.png"))
        settings_btn.setIconSize(QSize(20, 20))
        settings_btn.clicked.connect(self.show_settings_window)

        status_btn = QPushButton("", self)
        status_btn.setIcon(QIcon(os.path.dirname(os.path.abspath(__file__)) + "/resources/icons/status_ok.png"))
        status_btn.setIconSize(QSize(20, 20))

        self.statusBar().setStyleSheet("QStatusBar {border: 0; background-color: #FFF8DC;}")

        self.statusBar().addWidget(settings_btn)
        self.statusBar().addPermanentWidget(VLine())
        self.statusBar().addPermanentWidget(status_lbl)
        self.statusBar().addPermanentWidget(status_btn)
        self.statusBar().addPermanentWidget(VLine())

    def show_settings_window(self):
        self.settings_window.show()

    def set_active_btn_clicked(self):
        req = SatsActiveSet.Request()
        req.is_on = (self.set_active_btn.text() == "Set active")

        if req.is_on:
            self.rid = self.tle_list_widget.selectedItems()[0]
            req.id = int(self.rid.statusTip())

        if not self.background_color:
            self.background_color = self.rid.background()

        while not subs_and_clients.sat_set_active_client.wait_for_service(timeout_sec=1.0):
            # TODO LOADING
            print('Service sat_set_active_client is not available, waiting...')

        future = subs_and_clients.sat_set_active_client.call_async(req)
        while rclpy.ok():
            # TODO LOADING
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    QMessageBox.warning(self, "sat_set_active_client", "Cannot set active satellite.\n"
                                                                       "Stacktrace: {}".format(e),
                                        QMessageBox.Ok)
                else:
                    if response.res:
                        if self.set_active_btn.text() == "Set active":
                            self.set_active_btn.setText("Set inactive")
                            self.rid.setBackground(QColor("#9ee99e"))
                            self.antenna_graph_widget.is_tracking = True
                        else:
                            self.set_active_btn.setText("Set active")
                            self.rid.setBackground(self.background_color)
                            self.antenna_graph_widget.is_tracking = False
                            subs_and_clients.sat_azs, subs_and_clients.sat_els = [], []
                    else:
                        QMessageBox.warning(self, "sat_set_active_client", "Cannot set active satellite.",
                                            QMessageBox.Ok)
                break

    def update_settings(self):
        self.update_timer()
        self.antenna_adjustment_widget.update_steps()
        self.antenna_video_widget.update_thread()
        self.map_widget.update_mcc_qpoint()

    def closeEvent(self, event):
        req = SysDeauth.Request()
        while not subs_and_clients.sys_deauth_client.wait_for_service(timeout_sec=1.0):
            # TODO LOADING
            print('Service sys_deauth_client is not available, waiting...')

        future = subs_and_clients.sys_deauth_client.call_async(req)
        while rclpy.ok():
            # TODO LOADING
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    QMessageBox.warning(self, "sys_deauth_client", "Cannot deauthorize.\n"
                                                                   "Stacktrace: {}".format(e), QMessageBox.Ok)
                else:
                    if response.res:
                        subs_and_clients.destroy_node()
                        rclpy.shutdown()
                    else:
                        QMessageBox.warning(self, "sys_deauth_client", "Cannot deauthorize.", QMessageBox.Ok)
                break

    def check_active_satellite(self):
        req = SatsActive.Request()

        while not subs_and_clients.sat_active_client.wait_for_service(timeout_sec=1.0):
            # TODO LOADING
            print('Service sat_upd_client is not available, waiting...')

        future = subs_and_clients.sat_active_client.call_async(req)
        while rclpy.ok():
            # TODO LOADING
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    QMessageBox.warning(self, "sat_upd_client", "Cannot check for active satellite.\n"
                                                                "Stacktrace: {}".format(e), QMessageBox.Ok)
                else:
                    if response.is_on:
                        for i in range(self.tle_list_widget.count()):
                            if int(self.tle_list_widget.item(i).statusTip()) == response.id:
                                self.rid = self.tle_list_widget.item(i)
                                self.background_color = self.rid.background()
                                self.tle_list_widget.item(i).setBackground(QColor("#9ee99e"))
                                self.set_active_btn.setText("Set inactive")
                                self.antenna_graph_widget.is_tracking = True
                break

    # def login(self):


if __name__ == "__main__":
    app = QApplication(sys.argv)

    rclpy.init()
    subs_and_clients = SubscribersAndClients()
    spin_thread = Thread(target=rclpy.spin, args=(subs_and_clients,))
    spin_thread.start()

    main = MainWindow()
    main.show()

    sys.exit(app.exec_())
