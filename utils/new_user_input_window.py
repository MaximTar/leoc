from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QGridLayout, QWidget, QLabel, QSpinBox, QDoubleSpinBox, QMainWindow, QLineEdit, QCheckBox

# from utils.prediction_window import PredictionWindow
from antenna_interfaces.srv import SysPasswd

from utils.prediction_window_old import PredictionWindow
from utils.srv_client_handler import srv_ready
from utils.tle_handler import *


# noinspection PyUnresolvedReferences
class NewUserInputWindow(QMainWindow):
    def __init__(self, subs_and_clients=None, prnt=None):
        super(NewUserInputWindow, self).__init__(prnt)
        self.prnt = prnt
        self.setWindowTitle("Prediction")
        self.setWindowModality(Qt.ApplicationModal)

        self.subs_and_clients = subs_and_clients

        login_lbl = QLabel("Login:")
        password_lbl = QLabel("Password:")
        admin_lbl = QLabel("Admin:")

        self.login_le = QLineEdit()
        self.password_le = QLineEdit()
        self.admin_checkbox = QCheckBox()

        predict_btn = QPushButton("Add User")
        predict_btn.clicked.connect(self.add_user_btn_clicked)

        layout = QGridLayout()
        layout.addWidget(login_lbl, 0, 0)
        layout.addWidget(self.login_le, 0, 1)
        layout.addWidget(password_lbl, 1, 0)
        layout.addWidget(self.password_le, 1, 1)
        layout.addWidget(admin_lbl, 2, 0)
        layout.addWidget(self.admin_checkbox, 2, 1)
        layout.addWidget(predict_btn, 3, 0, 1, 2)

        central_widget = QWidget()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

    def centerize(self, prnt):
        if prnt:
            self.move(prnt.window().frameGeometry().topLeft() + prnt.window().rect().center() - self.rect().center())

    def add_user_btn_clicked(self):
        if self.subs_and_clients and srv_ready(self.subs_and_clients.sys_passwd_client):
            req = SysPasswd.Request()
            req.login = self.login_le.text()
            req.password = self.password_le.text()
            req.is_admin = self.admin_checkbox.isChecked()
            future = self.subs_and_clients.sys_passwd_client.call_async(req)
            while rclpy.ok():
                # TODO LOADING
                if future.done():
                    try:
                        response = future.result()
                    except Exception as e:
                        QMessageBox.warning(self, "sys_passwd_client", "Cannot add user {}.\n"
                                                                       "Stacktrace: {}".format(req.login, e),
                                            QMessageBox.Ok)
                    else:
                        if response.users and response.users[0] == req.login:
                            QMessageBox.information(self, "sys_passwd_client",
                                                    "Added user: {}.".format(req.login), QMessageBox.Ok)
                            self.close()
                            self.prnt.prnt.show_users_window()
                        else:
                            QMessageBox.warning(self, "sys_passwd_client", "Cannot add user {}.\n".format(req.login),
                                                QMessageBox.Ok)
                    break
