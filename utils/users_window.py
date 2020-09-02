import datetime

import rclpy
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QBrush, QColor
from PyQt5.QtWidgets import QMainWindow, QTableWidget, QLabel, QTableWidgetItem, QPushButton, QMessageBox, QVBoxLayout, QInputDialog, QLineEdit
from antenna_interfaces.srv import SysPasswd

from utils.srv_client_handler import srv_ready
from utils.new_user_input_window import NewUserInputWindow


# noinspection PyUnresolvedReferences
class UsersWindow(QMainWindow):
    def __init__(self, users=None, subs_and_clients=None, prnt=None):
        super().__init__()
        self.setWindowTitle("Users")
        self.setWindowModality(Qt.ApplicationModal)

        self.subs_and_clients = subs_and_clients
        self.prnt = prnt

        table_widget = QTableWidget(self)

        if users:
            table_layout = QVBoxLayout()
            inner_table_widget = QTableWidget(self)
            inner_table_widget.setColumnCount(3)
            rows = len(users)
            inner_table_widget.setRowCount(rows)
            inner_table_widget.setHorizontalHeaderLabels(["Username", "", ""])
            for i in range(0, rows):
                user = users[i]
                uname_item = QTableWidgetItem(user)
                uname_item.setFlags(Qt.ItemIsEditable)
                inner_table_widget.setItem(i, 0, uname_item)

                del_btn = QPushButton('Delete', self)
                del_btn.setToolTip(user)
                del_btn.clicked.connect(lambda ch, text=del_btn.toolTip(): self.delete_user(text))
                inner_table_widget.setCellWidget(i, 1, del_btn)

                pwd_btn = QPushButton('Password', self)
                pwd_btn.setToolTip(user)
                pwd_btn.clicked.connect(lambda ch, text=pwd_btn.toolTip(): self.change_password(text))
                inner_table_widget.setCellWidget(i, 2, pwd_btn)

            inner_table_widget.resizeColumnsToContents()

            add_user_btn = QPushButton("Add user", self)
            add_user_btn.clicked.connect(self.add_user_btn_clicked)

            table_layout.addWidget(inner_table_widget)
            table_layout.addWidget(add_user_btn)
            table_widget.setLayout(table_layout)
        else:
            table_widget = QLabel("No users", self)
            table_widget.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        self.setCentralWidget(table_widget)
        self.resize(500, 200)

        # if it will be a local variable - it'll destroyed at the end of the function
        self.new_user_win = None

    def centerize(self, prnt):
        if prnt:
            self.move(
                prnt.window().frameGeometry().topLeft() + prnt.window().rect().center() - self.rect().center())

    def delete_user(self, user_name):
        if self.subs_and_clients and srv_ready(self.subs_and_clients.sys_passwd_client):
            req = SysPasswd.Request()
            req.login = user_name
            future = self.subs_and_clients.sys_passwd_client.call_async(req)
            while rclpy.ok():
                # TODO LOADING
                if future.done():
                    try:
                        response = future.result()
                    except Exception as e:
                        QMessageBox.warning(self, "sys_passwd_client", "Cannot delete user {}.\n"
                                                                       "Stacktrace: {}".format(req.login, e),
                                            QMessageBox.Ok)
                    else:
                        if response.users and response.users[0] == req.login:
                            QMessageBox.information(self, "sys_passwd_client",
                                                    "Deleted user: {}.".format(req.login), QMessageBox.Ok)
                            self.close()
                            self.prnt.prnt.show_users_window()
                        else:
                            QMessageBox.warning(self, "sys_passwd_client", "Cannot delete user {}.\n".format(req.login),
                                                QMessageBox.Ok)
                    break

    def change_password(self, user_name):
        new_pwd, ok = QInputDialog.getText(self, "Change password", "New password:", QLineEdit.Normal, "")
        if ok and new_pwd != '':
            if self.subs_and_clients and srv_ready(self.subs_and_clients.sys_passwd_client):
                req = SysPasswd.Request()
                req.login = user_name
                req.password = new_pwd
                future = self.subs_and_clients.sys_passwd_client.call_async(req)
                while rclpy.ok():
                    # TODO LOADING
                    if future.done():
                        try:
                            response = future.result()
                        except Exception as e:
                            QMessageBox.warning(self, "sys_passwd_client", "Cannot change {}'s password.\n"
                                                                           "Stacktrace: {}".format(req.login, e),
                                                QMessageBox.Ok)
                        else:
                            if response.users and response.users[0] == req.login:
                                QMessageBox.information(self, "sys_passwd_client",
                                                        "Changed {}'s password.".format(req.login), QMessageBox.Ok)
                                # self.close()
                                # self.prnt.prnt.show_users_window()
                            else:
                                QMessageBox.warning(self, "sys_passwd_client",
                                                    "Cannot change {}'s password.\n".format(req.login),
                                                    QMessageBox.Ok)
                        break

    def add_user_btn_clicked(self):
        self.close()
        self.new_user_win = NewUserInputWindow(subs_and_clients=self.subs_and_clients, prnt=self)
        self.new_user_win.centerize(self.prnt)
        self.new_user_win.show()
