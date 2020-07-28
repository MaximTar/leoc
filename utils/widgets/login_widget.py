from PyQt5.QtCore import QSettings, Qt
from PyQt5.QtWidgets import QWidget, QPushButton, QLabel, QLineEdit, QGridLayout, QMessageBox


# noinspection PyUnresolvedReferences
class LoginWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Login Form')
        self.setWindowModality(Qt.ApplicationModal)
        self.resize(500, 120)

        layout = QGridLayout()

        # label_name = QLabel('<font size="4"> Username </font>')
        label_name = QLabel('Username')
        self.lineEdit_username = QLineEdit()
        self.lineEdit_username.setPlaceholderText('Please enter your username')
        layout.addWidget(label_name, 0, 0)
        layout.addWidget(self.lineEdit_username, 0, 1)

        # label_password = QLabel('<font size="4"> Password </font>')
        label_password = QLabel('Password')
        self.lineEdit_password = QLineEdit()
        self.lineEdit_password.setPlaceholderText('Please enter your password')
        layout.addWidget(label_password, 1, 0)
        layout.addWidget(self.lineEdit_password, 1, 1)

        button_login = QPushButton('Login')
        button_login.clicked.connect(self.check_password)
        layout.addWidget(button_login, 2, 0, 1, 2)
        layout.setRowMinimumHeight(2, 75)

        self.setLayout(layout)

    def check_password(self):
        pass
        # https://stackoverflow.com/questions/11812000/login-dialog-pyqt
        # msg = QMessageBox()
        #
        # if self.lineEdit_username.text() == 'Usernmae' and self.lineEdit_password.text() == '000':
        #     msg.setText('Success')
        #     msg.exec_()
        # else:
        #     msg.setText('Incorrect Password')
        #     msg.exec_()
