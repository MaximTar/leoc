import sys
from threading import Thread

from PyQt5.QtCore import QTimer, QSize, Qt
from PyQt5.QtGui import QIcon, QColor
from PyQt5.QtWidgets import *
from antenna_interfaces.srv import *

from utils.lines import *
from utils.settings_window import SettingsWindow
from utils.parameters_window import ParametersWindow
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
from utils.widgets.prediction_input_widget import PredictionInputWidget
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

        self.icon_ok = QIcon(os.path.dirname(os.path.abspath(__file__)) + "/resources/icons/status_ok.png")
        self.icon_fail = QIcon(os.path.dirname(os.path.abspath(__file__)) + "/resources/icons/status_fail.png")

        main_hbox = QHBoxLayout()
        main_hbox.setContentsMargins(10, 10, 10, 0)

        self.set_active_btn = QPushButton("Set active", self)
        self.set_active_btn.setToolTip("Set active satellite")
        self.set_active_btn.clicked.connect(self.set_active_btn_clicked)
        self.set_active_btn.setEnabled(False)

        self.dt_status = self.Status.NOT_OK

        self.status_combo_box = QComboBox(self)
        self.create_status_bar()

        self.prediction_window = PredictionInputWidget(subs_and_clients, parent=self)

        # TODO REMOVE
        self.settings_window = SettingsWindow(parent_upd_slot=self.update_settings)
        self.settings = self.settings_window.settings

        self.parameters_window = ParametersWindow(subs_and_clients, parent=self)

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
        subs_and_clients.set_status_slot(self.update_status_combo_box)

        self.login_widget = LoginWidget(subs_and_clients)
        # self.login_widget.show()

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
        by_id_btn = add_box.addButton("By ID", QMessageBox.ActionRole)
        by_name_btn = add_box.addButton("By name", QMessageBox.ActionRole)
        manual_btn = add_box.addButton("Set user DB", QMessageBox.ActionRole)
        add_box.setDefaultButton(QMessageBox.Cancel)

        add_box.exec()

        if add_box.clickedButton() == by_id_btn:
            sat_name, ok = QInputDialog.getInt(self, "Add TLE by satellite ID", "Satellite ID", min=0)
            if ok:
                req = SatsAdd.Request()
                req.num = str(sat_name)
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
                            QMessageBox.warning(self, "sat_add_client", "Cannot add satellite {} to list.\n"
                                                                        "Stacktrace: {}".format(req.num, e),
                                                QMessageBox.Ok)
                        else:
                            if response.res:
                                self.tle_list_widget.update_list()
                            else:
                                QMessageBox.warning(self, "sat_add_client",
                                                    "Cannot add satellite {} to list.".format(req.num),
                                                    QMessageBox.Ok)
                        break
        elif add_box.clickedButton() == by_name_btn:
            sat_name, ok = QInputDialog.getText(self, "Add TLE by satellite name", "Satellite name")
            if ok:
                self.add_sat_to_list_by_name(sat_name)
        elif add_box.clickedButton() == manual_btn:
            req = TlesUserSet.Request()
            f_name = QFileDialog.getOpenFileName(self, 'Open file', "Text files (*.txt)")[0]
            f = open(f_name, 'r')
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
                            reply_btn = QMessageBox.question(self, "Done", "TLEs added to user database.\n"
                                                                           "Do you want add it to the list?",
                                                             QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
                            if reply_btn == QMessageBox.Yes:
                                for fn in data.splitlines()[0::3]:
                                    fn = fn.rstrip().split(' ')
                                    self.add_sat_to_list_by_name(' '.join(fn))
                        else:
                            QMessageBox.warning(self, "sat_add_client", "Cannot add satellite to list.", QMessageBox.Ok)
                    break

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

    def predict_btn_clicked(self):
        self.prediction_window.centerize(self)
        self.prediction_window.show()

    def create_status_bar(self):
        # self.statusBar().showMessage("")
        status_lbl = QLabel("Status: ")

        status_combo_box_list = [" All", " ESC 1", " ESC 2", " Encoder 1", " Encoder 2", " Az Lim", " El Lim",
                                 " Lim Switch 1", " Lim Switch 2", " Lim Switch 3", " Lim Switch 4", " Data"]

        settings_btn = QPushButton("", self)
        settings_btn.setIcon(QIcon(os.path.dirname(os.path.abspath(__file__)) + "/resources/icons/settings.png"))
        settings_btn.setIconSize(QSize(20, 20))
        settings_btn.clicked.connect(self.show_parameters_window)

        for i in status_combo_box_list:
            self.status_combo_box.addItem(self.icon_ok, i)

        self.status_combo_box.adjustSize()
        self.status_combo_box.setMaxVisibleItems(12)

        self.statusBar().setStyleSheet("QStatusBar {border: 0; background-color: #FFF8DC;}")

        self.statusBar().addWidget(settings_btn)
        self.statusBar().addPermanentWidget(VLine())
        self.statusBar().addPermanentWidget(status_lbl)
        self.statusBar().addPermanentWidget(self.status_combo_box)
        self.statusBar().addPermanentWidget(VLine())

    def update_status_combo_box(self, mask):
        for idx in range(len(mask)):
            status = mask[idx]
            icon = self.icon_ok if status == '0' else self.icon_fail
            self.status_combo_box.setItemIcon(idx, icon)

    def show_settings_window(self):
        self.settings_window.show()

    def show_parameters_window(self):
        self.parameters_window.centerize(self)
        self.parameters_window.show()

    def set_active_btn_clicked(self):
        if self.tle_list_widget.selectedIndexes() or (self.set_active_btn.text() == "Set inactive"):
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
        else:
            QMessageBox.information(self, "Prediction", "Select satellite first!", QMessageBox.Ok)

    def update_settings(self):
        self.update_timer()
        self.antenna_adjustment_widget.update_steps()
        self.antenna_video_widget.update_thread()
        self.map_widget.update_mcc_qpoint()

    # def closeEvent(self, event):
    #     req = SysDeauth.Request()
    #     while not subs_and_clients.sys_deauth_client.wait_for_service(timeout_sec=1.0):
    #         # TODO LOADING
    #         print('Service sys_deauth_client is not available, waiting...')
    #
    #     future = subs_and_clients.sys_deauth_client.call_async(req)
    #     while rclpy.ok():
    #         # TODO LOADING
    #         if future.done():
    #             try:
    #                 response = future.result()
    #             except Exception as e:
    #                 QMessageBox.warning(self, "sys_deauth_client", "Cannot deauthorize.\n"
    #                                                                "Stacktrace: {}".format(e), QMessageBox.Ok)
    #             else:
    #                 if response.res:
    #                     subs_and_clients.destroy_node()
    #                     rclpy.shutdown()
    #                 else:
    #                     QMessageBox.warning(self, "sys_deauth_client", "Cannot deauthorize.", QMessageBox.Ok)
    #             break

    def add_sat_to_list_by_name(self, sat_name):
        req = SatsAdd.Request()
        req.name = str(sat_name)
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
                    QMessageBox.warning(self, "sat_add_client", "Cannot add satellite {} to list.\n"
                                                                "Stacktrace: {}".format(req.name, e),
                                        QMessageBox.Ok)
                else:
                    if response.res:
                        self.tle_list_widget.update_list()
                    else:
                        QMessageBox.warning(self, "sat_add_client",
                                            "Cannot add satellite {} to list.".format(req.name),
                                            QMessageBox.Ok)
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


if __name__ == "__main__":
    app = QApplication(sys.argv)

    rclpy.init()
    subs_and_clients = SubscribersAndClients()
    spin_thread = Thread(target=rclpy.spin, args=(subs_and_clients,))
    spin_thread.start()

    main = MainWindow()
    main.show()

    sys.exit(app.exec_())
