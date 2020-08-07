import rclpy
from PyQt5.QtCore import Qt, QPropertyAnimation, QEasingCurve
from PyQt5.QtWidgets import QWidget, QPushButton, QLabel, QLineEdit, QGridLayout, QMessageBox, QDesktopWidget
from antenna_interfaces.srv import SysAuth

from utils.srv_client_handler import srv_ready


# noinspection PyUnresolvedReferences
class LoginWidget(QWidget):
    def __init__(self, subs_and_clients):
        super().__init__()
        self.setWindowTitle('Login Form')

        self.setWindowModality(Qt.ApplicationModal)
        self.setWindowFlags(Qt.CustomizeWindowHint)

        self.resize(500, 120)

        layout = QGridLayout()

        label_name = QLabel('Username')
        self.line_edit_username = QLineEdit()
        self.line_edit_username.setPlaceholderText('Please enter your username')
        layout.addWidget(label_name, 0, 0)
        layout.addWidget(self.line_edit_username, 0, 1)

        label_password = QLabel('Password')
        self.line_edit_password = QLineEdit()
        self.line_edit_password.setPlaceholderText('Please enter your password')
        self.line_edit_password.setEchoMode(QLineEdit.Password)
        layout.addWidget(label_password, 1, 0)
        layout.addWidget(self.line_edit_password, 1, 1)

        self.wrong_lbl = QLabel("Wrong login/password")
        self.wrong_lbl.setVisible(False)
        self.wrong_lbl.setStyleSheet('color: red')
        layout.addWidget(self.wrong_lbl, 2, 0, 1, 2, Qt.AlignHCenter)

        button_login = QPushButton('Login')
        button_login.clicked.connect(self.check_password)
        layout.addWidget(button_login, 3, 0, 1, 2)

        self.setLayout(layout)
        self.center()

        self.subs_and_clients = subs_and_clients
        self.is_admin = False

        self.centered_lbl_geometry, self.left_lbl_geometry, self.right_lbl_geometry = None, None, None
        self.animation = None

    def center(self):
        ag = QDesktopWidget().availableGeometry()
        sg = QDesktopWidget().screenGeometry()

        widget = self.geometry()
        x = ag.width() / 2 - widget.width() / 2
        y = ag.height() - sg.height() / 2 - widget.height() / 2
        self.move(x, y)

    def set_animation(self):
        self.centered_lbl_geometry = self.wrong_lbl.geometry()
        self.wrong_lbl.setGeometry(self.centered_lbl_geometry.x() - 10, self.wrong_lbl.geometry().y(),
                                   self.wrong_lbl.geometry().width(), self.wrong_lbl.geometry().height())
        self.left_lbl_geometry = self.wrong_lbl.geometry()
        self.wrong_lbl.setGeometry(self.centered_lbl_geometry.x() + 10, self.wrong_lbl.geometry().y(),
                                   self.wrong_lbl.geometry().width(), self.wrong_lbl.geometry().height())
        self.right_lbl_geometry = self.wrong_lbl.geometry()
        self.wrong_lbl.setGeometry(self.centered_lbl_geometry)

        self.animation = QPropertyAnimation(self.wrong_lbl, b"geometry")
        self.animation.setDuration(100)
        self.animation.setLoopCount(2)
        self.animation.setEasingCurve(QEasingCurve.InOutQuint)
        self.animation.setStartValue(self.centered_lbl_geometry)
        self.animation.setEndValue(self.centered_lbl_geometry)
        self.animation.setKeyValueAt(0.333, self.left_lbl_geometry)
        self.animation.setKeyValueAt(0.666, self.right_lbl_geometry)

    def check_password(self):
        if srv_ready(self.subs_and_clients.sys_auth_client):
            req = SysAuth.Request()
            req.login = str(self.line_edit_username.text())
            req.password = str(self.line_edit_password.text())
            future = self.subs_and_clients.sys_auth_client.call_async(req)
            while rclpy.ok():
                # TODO LOADING
                if future.done():
                    try:
                        response = future.result()
                    except Exception as e:
                        QMessageBox.warning(self, "sys_auth_client", "Cannot login.\n"
                                                                     "Stacktrace: {}".format(e), QMessageBox.Ok)
                    else:
                        if response.res:
                            self.close()
                            self.is_admin = response.is_admin
                        else:
                            self.line_edit_username.clear()
                            self.line_edit_password.clear()
                            self.wrong_lbl.setVisible(True)
                            self.set_animation()
                            self.animation.start()
                    break
