import datetime
from threading import Thread

from PyQt5.QtCore import QTimer, QSize, Qt, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QIcon, QColor
from PyQt5.QtWidgets import *
from antenna_interfaces.srv import *

from utils.lines import *
from utils.parameters_window import ParametersWindow
from utils.settings_window import SettingsWindow
from utils.subs_and_clients import SubscribersAndClients
from utils.tle_handler import *
from utils.user_db_window import UserDbWindow
from utils.widgets.antenna_adjustment_widget import AntennaAdjustmentWidget
from utils.widgets.antenna_control_widget import AntennaControlWidget
from utils.widgets.antenna_graph_widget import AntennaGraphWidget
from utils.widgets.antenna_pose_widget import AntennaPoseWidget
from utils.widgets.antenna_velocities_widget import AntennaVelocitiesWidget
from utils.widgets.antenna_video_widget import AntennaVideoWidget
from utils.widgets.login_widget import LoginWidget
from utils.widgets.map_widget import MapWidget
from utils.widgets.pose_diff_graph_widget import PoseDiffGraphWidget
from utils.widgets.prediction_input_widget import PredictionInputWidget
from utils.widgets.satellite_data_widget import SatelliteDataWidget
from utils.widgets.tle_list_widget import TleListWidget
from utils.widgets.velocity_diff_graph_widget import VelocityDiffGraphWidget


# TODO rosout to file
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
# noinspection PyUnresolvedReferences,PyCallByClass,PyArgumentList,PyTypeChecker
class MainWindow(QMainWindow):
    timer_signal = pyqtSignal()

    class Status(Enum):
        OK = 1
        NOT_OK = 0

    def __init__(self):
        super().__init__()
        self.setWindowTitle("LEOC")
        # TODO MAKE FUNCTIONAL TO RELOGIN

        self.timestamp = None
        self.login_widget = LoginWidget(subs_and_clients, prnt=self)
        self.login_widget.show()

        self.icon_ok = QIcon(os.path.dirname(os.path.abspath(__file__)) + "/resources/icons/status_ok.png")
        self.icon_fail = QIcon(os.path.dirname(os.path.abspath(__file__)) + "/resources/icons/status_fail.png")
        self.background_color = QColor("#fcfcfc")

        self.set_active_btn = QPushButton("Track", self)
        self.set_active_btn.setToolTip("Track satellite")
        self.set_active_btn.clicked.connect(self.set_active_btn_clicked)
        # self.set_active_btn.setEnabled(True)

        self.dt_status = self.Status.NOT_OK

        self.prediction_window = PredictionInputWidget(subs_and_clients, parent=self)

        # TODO REMOVE
        self.settings_window = SettingsWindow(parent_upd_slot=self.update_settings)
        self.settings = self.settings_window.settings

        self.parameters_window = ParametersWindow(subs_and_clients, parent=self)

        self.map_widget = MapWidget(settings_window=self.settings_window, ts=self.timestamp)

        self.satellite_data_widget = SatelliteDataWidget(settings=self.settings)
        self.antenna_graph_widget = AntennaGraphWidget()
        self.pose_graph_widget = PoseDiffGraphWidget()
        self.antenna_pose_widget = AntennaPoseWidget()
        self.antenna_pose_widget.set_pose_graph_slot(self.pose_graph_widget.update_graph)
        self.velocity_graph_widget = VelocityDiffGraphWidget()
        self.antenna_velocities_widget = AntennaVelocitiesWidget()
        self.antenna_velocities_widget.set_vel_graph_slot(self.velocity_graph_widget.update_graph)
        self.antenna_adjustment_widget = AntennaAdjustmentWidget(settings=self.settings,
                                                                 subs_and_clients=subs_and_clients)
        self.antenna_control_widget = AntennaControlWidget(subs_and_clients)
        # self.antenna_time_widget = AntennaTimeWidget(dt_slot=self.set_dt)
        self.antenna_video_widget = AntennaVideoWidget(self.settings)

        self.dt = None

        # self.construct_widgets()

        self.rid = None
        subs_and_clients.set_ant_vel_slot(self.antenna_velocities_widget.update_velocity)
        subs_and_clients.set_sat_graph_slot(self.antenna_graph_widget.update_sat_graph)
        subs_and_clients.set_ant_graph_slot(self.antenna_graph_widget.update_ant_graph)
        subs_and_clients.set_ant_pose_slot(self.antenna_pose_widget.update_pose)
        subs_and_clients.set_status_slot(self.update_status_combo_box)

        # self.prediction_window = PredictionWindow()

        self.timer_signal.connect(self.start_heartbeat_timer)
        self.clock_lbl = QLabel()
        self.status_combo_box = QComboBox(self)
        # self.create_status_bar()
        subs_and_clients.set_clock_slot(self.update_clock_lbl)
        self.heartbeat_timer = QTimer()
        self.heartbeat_timer.timeout.connect(self.show_server_error)

        self.user_db_win = UserDbWindow()
        self.server_error_box = None

    def after_login(self):
        self.tle_list_widget = TleListWidget(subs_and_clients, parent_slot=self.update_map_widget,
                                             data_slot=self.update_sat_data)
        self.map_widget.set_uncheck_list_and_slot(self.tle_list_widget.checked_indices_list,
                                                  self.tle_list_widget.uncheck_item)
        self.check_active_satellite()

        get_tles(self.tle_list_widget, subs_and_clients, on_startup=True)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_map_widget)
        self.timer.timeout.connect(self.update_sat_data)
        self.timer.start(int(self.settings.value("general_settings/map_update_period", 1000)))

    def construct_widgets(self, is_admin=False):
        main_hbox = QHBoxLayout()
        main_hbox.setContentsMargins(10, 10, 10, 0)

        splitter = QSplitter(Qt.Horizontal)

        lw = self.construct_left_widget()
        lw.setMinimumSize(250, 400)
        splitter.addWidget(lw)

        cw = self.construct_central_widget(is_admin)
        splitter.addWidget(cw)

        rw = self.construct_right_widget(is_admin)
        rw.setMinimumSize(200, 400)
        splitter.addWidget(rw)

        splitter.setSizes([250, 640, 150])
        main_hbox.addWidget(splitter)

        # noinspection PyArgumentList
        central_widget = QWidget()
        central_widget.setLayout(main_hbox)
        self.setCentralWidget(central_widget)

    def construct_central_widget(self, is_admin=False):
        splitter = QSplitter(Qt.Vertical)
        mw = self.map_widget
        splitter.addWidget(mw)

        if is_admin:
            splitter.addWidget(self.pose_graph_widget)
            splitter.addWidget(self.velocity_graph_widget)
            splitter.setStretchFactor(0, 100)
            splitter.setStretchFactor(1, 1)
            splitter.setStretchFactor(2, 1)

        return splitter

    @pyqtSlot()
    def start_heartbeat_timer(self):
        self.heartbeat_timer.start(2000)

    def show_server_error(self):
        if self.server_error_box:
            self.server_error_box.close()
        self.server_error_box = QMessageBox(self)
        self.server_error_box.setIcon(QMessageBox.Critical)
        self.server_error_box.setWindowTitle("ERROR")
        self.server_error_box.setText("Lost server connection")
        self.server_error_box.setStandardButtons(QMessageBox.Ok)
        ca_btn = QPushButton("Close app")
        self.server_error_box.addButton(ca_btn, QMessageBox.ActionRole)
        self.server_error_box.exec()
        if self.server_error_box.clickedButton() == ca_btn:
            self.close()
            # noinspection PyProtectedMember,PyUnresolvedReferences
            os._exit(0)
            # sys.exit()

    def update_timer(self):
        self.timer.start(int(self.settings.value("general_settings/map_update_period", 1000)))

    # def set_dt(self, dt):
    #     if dt:
    #         self.dt = dt
    #         self.dt_status = self.Status.OK
    #         self.update_set_btn_state()

    def update_map_widget(self):
        tle_list = get_tle_list_by_indices(self.tle_list_widget.checked_indices_list)
        orb_list = get_orb_list_by_tle_list(tle_list)
        self.map_widget.update_map(orb_list)

    def update_sat_data(self):
        if self.tle_list_widget.selectedIndexes():
            tle = get_tle_by_index(self.tle_list_widget.selectedIndexes()[0].row())
            orb = get_orb_by_tle(tle)
            self.satellite_data_widget.update_data(orb, self.timestamp)

    def remove_from_tle_list_widget(self, index):
        remove_tle_by_index(index)
        self.tle_list_widget.update_list()
        get_tles(self.tle_list_widget, subs_and_clients)
        self.check_active_satellite()

    def construct_left_widget(self):
        clear_graphs_btn = QPushButton("Clear graph")
        clear_graphs_btn.clicked.connect(self.clear_graphs_btn_clicked)

        splitter = QSplitter(Qt.Vertical)
        splitter.setStretchFactor(1, 1)
        splitter.addWidget(clear_graphs_btn)
        splitter.addWidget(self.antenna_graph_widget)

        splitter.addWidget(self.antenna_pose_widget)
        splitter.addWidget(self.antenna_velocities_widget)

        splitter.addWidget(self.antenna_adjustment_widget)
        splitter.addWidget(self.antenna_video_widget)
        return splitter

    def clear_graphs_btn_clicked(self):
        subs_and_clients.sat_azs, subs_and_clients.sat_els = [], []
        subs_and_clients.ant_azs, subs_and_clients.ant_els = [], []
        self.antenna_pose_widget.clear_data()
        self.antenna_velocities_widget.clear_data()

    def construct_right_widget(self, is_admin=False):
        right_widget = QWidget()
        right_vbox = QVBoxLayout()
        right_vbox.setContentsMargins(5, 10, 10, 0)
        btn_widget = QWidget()
        two_btn_widget = QWidget()
        btn_grid = QGridLayout()
        two_btn_grid = QGridLayout()

        add_btn = QPushButton("Add", self)
        add_btn.setToolTip("Add TLE to the list")
        add_btn.clicked.connect(self.add_btn_clicked)

        remove_btn = QPushButton("Remove", self)
        remove_btn.setToolTip("Remove TLE from the list")
        remove_btn.clicked.connect(self.remove_btn_clicked)

        update_btn = QPushButton("Update", self)
        update_btn.setToolTip("Update TLE list")
        update_btn.clicked.connect(self.update_btn_clicked)

        predict_btn = QPushButton("Predict", self)
        predict_btn.setToolTip("Make prediction")
        predict_btn.clicked.connect(self.predict_btn_clicked)

        show_db_btn = QPushButton("Show user DB", self)
        show_db_btn.setToolTip("Show user database")
        show_db_btn.clicked.connect(self.show_db_btn_clicked)

        btn_grid.addWidget(predict_btn, 0, 0, 1, 10)
        btn_grid.addWidget(self.set_active_btn, 0, 10, 1, 10)
        btn_grid.addWidget(add_btn, 2, 0, 1, 6)
        btn_grid.addWidget(remove_btn, 2, 7, 1, 6)
        btn_grid.addWidget(update_btn, 2, 14, 1, 6)
        btn_grid.addWidget(show_db_btn, 3, 0, 1, 20)

        btn_widget.setLayout(btn_grid)

        home_btn = QPushButton("Home", self)
        home_btn.clicked.connect(self.home_btn_clicked)
        two_btn_grid.addWidget(home_btn, 0, 0, 1, 10)

        stop_btn = QPushButton("Stop", self)
        stop_btn.clicked.connect(self.stop_btn_clicked)
        two_btn_grid.addWidget(stop_btn, 0, 10, 1, 10)

        if is_admin:
            reset_btn = QPushButton("Reset", self)
            # reset_btn.setToolTip("Remove TLE from the list")
            reset_btn.clicked.connect(self.reset_btn_clicked)
            two_btn_grid.addWidget(reset_btn, 0, 20, 1, 10)
        two_btn_widget.setLayout(two_btn_grid)

        splitter = QSplitter(Qt.Vertical)
        splitter.addWidget(two_btn_widget)
        splitter.addWidget(self.antenna_control_widget)
        splitter.addWidget(btn_widget)
        splitter.addWidget(self.tle_list_widget)
        scroll_area = QScrollArea()
        scroll_area.setWidget(self.satellite_data_widget)
        splitter.addWidget(scroll_area)

        splitter.setStretchFactor(3, 8)
        splitter.setStretchFactor(4, 1)

        right_vbox.addWidget(splitter)
        # right_vbox.addWidget(btn_widget)
        right_widget.setLayout(right_vbox)
        return right_widget

    def home_btn_clicked(self):
        if srv_ready(subs_and_clients.params_set_client):
            req = ParamsSet.Request()
            req.names = ["op_mode"]
            req.values = ["3"]
            future = subs_and_clients.params_set_client.call_async(req)
            while rclpy.ok():
                # TODO LOADING
                if future.done():
                    try:
                        response = future.result()
                    except Exception as e:
                        QMessageBox.warning(self, "params_set_client",
                                            "Cannot change parameters.\n"
                                            "Stacktrace: {}".format(e),
                                            QMessageBox.Ok)
                    else:
                        if response.res:
                            QMessageBox.information(self, "params_set_client",
                                                    "Antenna moves to home position.",
                                                    QMessageBox.Ok)
                        else:
                            QMessageBox.warning(self, "params_set_client",
                                                "Cannot change parameters.", QMessageBox.Ok)
                    break

    def stop_btn_clicked(self):
        if srv_ready(subs_and_clients.params_set_client):
            req = ParamsSet.Request()
            req.names = ["op_mode"]
            req.values = ["0"]
            future = subs_and_clients.params_set_client.call_async(req)
            while rclpy.ok():
                # TODO LOADING
                if future.done():
                    try:
                        response = future.result()
                    except Exception as e:
                        QMessageBox.warning(self, "params_set_client", "Cannot stop antenna.\n"
                                                                       "Stacktrace: {}".format(e), QMessageBox.Ok)
                    else:
                        if response.res:
                            # TODO CHECK !!!
                            QMessageBox.information(self, "params_set_client", "Antenna stopped", QMessageBox.Ok)
                            self.set_active_btn.setText("Track")
                            self.rid.setBackground(self.background_color)
                            self.antenna_graph_widget.is_tracking = False
                            self.antenna_pose_widget.clear_labels()
                            self.antenna_velocities_widget.clear_labels()
                            subs_and_clients.sat_azs, subs_and_clients.sat_els = [], []
                            subs_and_clients.ant_azs, subs_and_clients.ant_els = [], []
                            self.antenna_pose_widget.clear_data()
                            self.antenna_velocities_widget.clear_data()
                        else:
                            QMessageBox.warning(self, "params_set_client", "Cannot stop antenna", QMessageBox.Ok)

                    break

    def reset_btn_clicked(self):
        if srv_ready(subs_and_clients.mcu_reset_client):
            req = McuReset.Request()
            future = subs_and_clients.mcu_reset_client.call_async(req)
            while rclpy.ok():
                # TODO LOADING
                if future.done():
                    try:
                        response = future.result()
                    except Exception as e:
                        QMessageBox.warning(self, "mcu_reset_client", "Cannot show user db.\n"
                                                                      "Stacktrace: {}".format(e),
                                            QMessageBox.Ok)
                    else:
                        if response.res:
                            QMessageBox.information(self, "mcu_reset_client",
                                                    "MCU reset successful.",
                                                    QMessageBox.Ok)
                        else:
                            QMessageBox.warning(self, "mcu_reset_client",
                                                "Cannot reset MCU.",
                                                QMessageBox.Ok)
                    break

    def show_db_btn_clicked(self):
        if srv_ready(subs_and_clients.usr_tle_client):
            req = TlesUser.Request()
            future = subs_and_clients.usr_tle_client.call_async(req)
            while rclpy.ok():
                # TODO LOADING
                if future.done():
                    try:
                        response = future.result()
                    except Exception as e:
                        QMessageBox.warning(self, "usr_tle_client", "Cannot show user db.\n"
                                                                    "Stacktrace: {}".format(e),
                                            QMessageBox.Ok)
                    else:
                        if response.tles:
                            lines = response.tles.splitlines()
                            new_lines = [x for y in (lines[i:i + 3] + ['\n'] * (i < len(lines) - 2) for i in
                                                     range(0, len(lines), 3)) for x in y]
                            new_str = '\n'.join(new_lines)
                            self.user_db_win = UserDbWindow(new_str)
                            self.user_db_win.show()
                            self.user_db_win.centerize(self)
                        else:
                            QMessageBox.warning(self, "usr_tle_client",
                                                "Cannot show user db (it may be empty).",
                                                QMessageBox.Ok)
                    break

    # def update_set_btn_state(self):
    #     self.set_active_btn.setEnabled(bool(self.dt_status.value))

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
                if srv_ready(subs_and_clients.sat_add_client):
                    req = SatsAdd.Request()
                    req.num = str(sat_name)
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
                                    get_tles(self.tle_list_widget, subs_and_clients)
                                else:
                                    QMessageBox.warning(self, "sat_add_client",
                                                        "Cannot add satellite {} to list.".format(req.num),
                                                        QMessageBox.Ok)
                            break
        elif add_box.clickedButton() == by_name_btn:
            sat_name, ok = QInputDialog.getText(self, "Add TLE by satellite name", "Satellite name")
            if ok:
                if srv_ready(subs_and_clients.sat_add_client):
                    req = SatsAdd.Request()
                    req.name = str(sat_name)
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
                                    get_tles(self.tle_list_widget, subs_and_clients)
                                else:
                                    QMessageBox.warning(self, "sat_add_client",
                                                        "Cannot add satellite {} to list.".format(req.name),
                                                        QMessageBox.Ok)
                            break
        elif add_box.clickedButton() == manual_btn:
            if srv_ready(subs_and_clients.usr_tle_set_client):
                req = TlesUserSet.Request()
                f_name = QFileDialog.getOpenFileName(self, 'Open file', "Text files (*.txt)")[0]
                f = open(f_name, 'r')
                with f:
                    data = f.read()
                f.close()
                req.tles = data
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
                                QMessageBox.warning(self, "sat_add_client", "Cannot add satellite to list.",
                                                    QMessageBox.Ok)
                        break
            get_tles(self.tle_list_widget, subs_and_clients)
            self.check_active_satellite()

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
                if srv_ready(subs_and_clients.sat_del_client):
                    req = SatsDel.Request()
                    req.id = int(selected_items[0].statusTip())
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
                                    get_tles(self.tle_list_widget, subs_and_clients)
                                    self.check_active_satellite()
                                else:
                                    QMessageBox.warning(self, "sat_del_client", "Cannot delete satellite from list.",
                                                        QMessageBox.Ok)
                            break

    def update_btn_clicked(self):
        if srv_ready(subs_and_clients.sat_upd_client):
            req = SatsUpdate.Request()
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
                        if response.ids:
                            upd_list = []
                            for i in range(self.tle_list_widget.count()):
                                if int(self.tle_list_widget.item(i).statusTip()) in response.ids:
                                    upd_list.append(self.tle_list_widget.item(i).text())
                            if upd_list:
                                QMessageBox.information(self, "sat_upd_client",
                                                        "Updated TLEs: .".format(', '.join(upd_list)), QMessageBox.Ok)
                    break

    def predict_btn_clicked(self):
        if self.tle_list_widget.selectedIndexes():
            self.prediction_window.set_idx(self.tle_list_widget.selectedIndexes()[0].row())
            self.prediction_window.centerize(self)
            self.prediction_window.show()
        else:
            QMessageBox.information(self, "Prediction", "Select satellite first!", QMessageBox.Ok)

    def create_status_bar(self, is_admin=False):
        # self.statusBar().showMessage("")
        status_lbl = QLabel("Status: ")

        status_combo_box_list = [" All", " ESC 1", " ESC 2", " Encoder 1", " Encoder 2", " Az Lim", " El Lim",
                                 " Lim Switch 1", " Lim Switch 2", " Lim Switch 3", " Lim Switch 4", " Lim Switch 5",
                                 " Lim Switch 6", " Data"]

        for i in status_combo_box_list:
            self.status_combo_box.addItem(self.icon_ok, i)

        self.status_combo_box.adjustSize()
        self.status_combo_box.setMaxVisibleItems(14)

        self.statusBar().setStyleSheet("QStatusBar {border: 0; background-color: #FFF8DC;}")

        if is_admin:
            settings_btn = QPushButton("", self)
            settings_btn.setIcon(QIcon(os.path.dirname(os.path.abspath(__file__)) + "/resources/icons/settings.png"))
            settings_btn.setIconSize(QSize(20, 20))
            settings_btn.clicked.connect(self.show_parameters_window)
            self.statusBar().addWidget(settings_btn)

            # obs_pose_btn = QPushButton("Observer Position", self)
            # obs_pose_btn.clicked.connect(self.show_obs_pose_window)
            # self.statusBar().addWidget(obs_pose_btn)

            users_btn = QPushButton("Users", self)
            users_btn.clicked.connect(self.show_users_window)
            self.statusBar().addWidget(users_btn)

        self.statusBar().addWidget(self.clock_lbl)

        self.statusBar().addPermanentWidget(VLine())
        self.statusBar().addPermanentWidget(status_lbl)
        self.statusBar().addPermanentWidget(self.status_combo_box)
        self.statusBar().addPermanentWidget(VLine())

    def show_users_window(self):
        pass

    def show_obs_pose_window(self):
        pass

    def update_clock_lbl(self, ts):
        if self.server_error_box:
            self.server_error_box.close()
        self.timestamp = ts
        # date_time = datetime.datetime.utcfromtimestamp(float(ts))
        date_time = datetime.datetime.fromtimestamp(float(ts))
        self.clock_lbl.setText(date_time.strftime('%Y-%m-%d %H:%M:%S'))
        self.timer_signal.emit()

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
        if self.tle_list_widget.selectedIndexes() or (self.set_active_btn.text() == "Stop Tracking"):

            to_track = (self.set_active_btn.text() == "Track")

            if srv_ready(subs_and_clients.sat_set_active_client):
                req = SatsActiveSet.Request()
                req.is_on = to_track

                if req.is_on:
                    self.rid = self.tle_list_widget.selectedItems()[0]
                    req.id = int(self.rid.statusTip())

                future = subs_and_clients.sat_set_active_client.call_async(req)
                while rclpy.ok():
                    # TODO LOADING
                    if future.done():
                        try:
                            response = future.result()
                        except Exception as e:
                            QMessageBox.warning(self, "sat_set_active_client",
                                                "Cannot track satellite.\n"
                                                "Stacktrace: {}".format(e),
                                                QMessageBox.Ok)
                        else:
                            if response.res:
                                if srv_ready(subs_and_clients.params_set_client):
                                    req = ParamsSet.Request()
                                    req.names = ["op_mode"]
                                    req.values = ["1"] if to_track else ["0"]
                                    future = subs_and_clients.params_set_client.call_async(req)
                                    while rclpy.ok():
                                        # TODO LOADING
                                        if future.done():
                                            try:
                                                response = future.result()
                                            except Exception as e:
                                                QMessageBox.warning(self, "params_set_client",
                                                                    "Cannot change parameters.\n"
                                                                    "Stacktrace: {}".format(e), QMessageBox.Ok)
                                            else:
                                                if response.res:
                                                    if self.set_active_btn.text() == "Track":
                                                        self.set_active_btn.setText("Stop Tracking")
                                                        self.rid.setBackground(QColor("#9ee99e"))
                                                        self.antenna_graph_widget.is_tracking = True
                                                    else:
                                                        self.set_active_btn.setText("Track")
                                                        self.rid.setBackground(self.background_color)
                                                        self.antenna_graph_widget.is_tracking = False
                                                        self.antenna_pose_widget.clear_labels()
                                                        self.antenna_velocities_widget.clear_labels()
                                                        subs_and_clients.sat_azs, subs_and_clients.sat_els = [], []
                                                        subs_and_clients.ant_azs, subs_and_clients.ant_els = [], []
                                                        self.antenna_pose_widget.clear_data()
                                                        self.antenna_velocities_widget.clear_data()
                                                else:
                                                    QMessageBox.warning(self, "params_set_client",
                                                                        "Cannot change parameters.",
                                                                        QMessageBox.Ok)
                                            break

                            else:
                                QMessageBox.warning(self, "sat_set_active_client",
                                                    "Cannot track satellite.",
                                                    QMessageBox.Ok)
                        break

            # elif srv_ready(subs_and_clients.params_set_client):
            #     req = ParamsSet.Request()
            #     req.names = ["op_mode"]
            #     req.values = ["1"] if to_track else ["0"]
            #     future = subs_and_clients.params_set_client.call_async(req)
            #     while rclpy.ok():
            #         # TODO LOADING
            #         if future.done():
            #             try:
            #                 response = future.result()
            #             except Exception as e:
            #                 QMessageBox.warning(self, "params_set_client",
            #                                     "Cannot change parameters.\n"
            #                                     "Stacktrace: {}".format(e), QMessageBox.Ok)
            #             else:
            #                 if response.res:
            #                     if self.set_active_btn.text() == "Track":
            #                         self.set_active_btn.setText("Stop Tracking")
            #                         self.rid.setBackground(QColor("#9ee99e"))
            #                         self.antenna_graph_widget.is_tracking = True
            #                     else:
            #                         self.set_active_btn.setText("Track")
            #                         self.rid.setBackground(self.background_color)
            #                         self.antenna_graph_widget.is_tracking = False
            #                         self.antenna_pose_widget.clear_labels()
            #                         self.antenna_velocities_widget.clear_labels()
            #                         subs_and_clients.sat_azs, subs_and_clients.sat_els = [], []
            #                         subs_and_clients.ant_azs, subs_and_clients.ant_els = [], []
            #                         self.antenna_pose_widget.clear_data()
            #                         self.antenna_velocities_widget.clear_data()
            #                 else:
            #                     QMessageBox.warning(self, "params_set_client",
            #                                         "Cannot change parameters.",
            #                                         QMessageBox.Ok)
            #                 break

        else:
            QMessageBox.information(self, "Tracking", "Select satellite first!", QMessageBox.Ok)

    def update_settings(self):
        self.update_timer()
        self.antenna_adjustment_widget.update_steps()
        self.antenna_video_widget.update_thread()
        self.map_widget.update_mcc_qpoint()

    def closeEvent(self, event):
        if srv_ready(subs_and_clients.sys_deauth_client):
            req = SysDeauth.Request()
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

    def add_sat_to_list_by_name(self, sat_name):
        if srv_ready(subs_and_clients.sat_add_client):
            req = SatsAdd.Request()
            req.name = str(sat_name)
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
                            get_tles(self.tle_list_widget, subs_and_clients)
                            self.check_active_satellite()
                        else:
                            QMessageBox.warning(self, "sat_add_client",
                                                "Cannot add satellite {} to list.".format(req.name),
                                                QMessageBox.Ok)
                    break

    def check_active_satellite(self):
        if srv_ready(subs_and_clients.sat_active_client):
            req = SatsActive.Request()
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
                                    self.tle_list_widget.item(i).setBackground(QColor("#9ee99e"))
                                    self.set_active_btn.setText("Stop Tracking")
                                    self.antenna_graph_widget.is_tracking = True
                    break


if __name__ == "__main__":
    app = QApplication(sys.argv)

    rclpy.init()
    subs_and_clients = SubscribersAndClients()
    spin_thread = Thread(target=rclpy.spin, args=(subs_and_clients,))
    spin_thread.start()

    main = MainWindow()
    # main.show()

    sys.exit(app.exec_())
