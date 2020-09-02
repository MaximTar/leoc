import rclpy
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QMainWindow, QLabel, QSpinBox, QTableWidget, QTableWidgetItem, QDoubleSpinBox, \
    QPushButton, QWidget, QVBoxLayout, QMessageBox
from antenna_interfaces.srv import Params, ParamsInfo, ParamsSet

from utils.srv_client_handler import srv_ready

pass_args = 0


# noinspection PyUnresolvedReferences
class ParametersWindow(QMainWindow):
    def __init__(self, subs_and_clients, parent=None):
        super().__init__(parent=parent)
        self.setWindowTitle("Parameters")
        self.setWindowModality(Qt.ApplicationModal)
        self.subs_and_clients = subs_and_clients

    def __init(self):
        if srv_ready(self.subs_and_clients.params_info_client):
            req = ParamsInfo.Request()
            future = self.subs_and_clients.params_info_client.call_async(req)
            while rclpy.ok():
                # TODO LOADING
                if future.done():
                    try:
                        response = future.result()
                    except Exception as e:
                        table_widget = QLabel("Cannot get parameters\n"
                                              "Stacktrace: {}".format(e), self)
                        table_widget.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
                    else:
                        if response.names:
                            self.names = response.names[pass_args:]
                            types = response.types[pass_args:]
                            descs = response.descs[pass_args:]
                            table_widget = QWidget(self)
                            table_layout = QVBoxLayout()
                            self.inner_table_widget = QTableWidget(self)
                            self.inner_table_widget.setColumnCount(4)
                            # n_rows = len(response.names)
                            n_rows = len(self.names)
                            self.inner_table_widget.setRowCount(n_rows)
                            self.inner_table_widget.setHorizontalHeaderLabels(["Name", "Type", "Value", "Description"])
                            for i in range(0, n_rows):
                                # n_item = QTableWidgetItem(response.names[i])
                                n_item = QTableWidgetItem(self.names[i])
                                # t_item = QTableWidgetItem(response.types[i])
                                t_item = QTableWidgetItem(types[i])
                                if types[i] == 'uint8':
                                    v_item = QSpinBox(self)
                                    v_item.setMinimum(-255)
                                    v_item.setMaximum(255)
                                elif types[i] == 'uint16':
                                    v_item = QSpinBox(self)
                                    v_item.setMinimum(-65535)
                                    v_item.setMaximum(65535)
                                elif types[i] == 'float':
                                    v_item = QDoubleSpinBox(self)
                                    v_item.setMinimum(-1000)
                                    v_item.setMaximum(1000)
                                else:
                                    v_item = QLabel("Call admin")
                                d_item = QTableWidgetItem(descs[i])
                                n_item.setFlags(Qt.ItemIsEditable)
                                t_item.setFlags(Qt.ItemIsEditable)
                                d_item.setFlags(Qt.ItemIsEditable)
                                self.inner_table_widget.setItem(i, 0, n_item)
                                self.inner_table_widget.setItem(i, 1, t_item)
                                self.inner_table_widget.setCellWidget(i, 2, v_item)
                                self.inner_table_widget.setItem(i, 3, d_item)
                            self.inner_table_widget.resizeColumnsToContents()

                            self.set_values()

                            apply_btn = QPushButton("Apply", self)
                            apply_btn.clicked.connect(self.apply_btn_clicked)

                            table_layout.addWidget(self.inner_table_widget)
                            table_layout.addWidget(apply_btn)
                            table_widget.setLayout(table_layout)
                        else:
                            table_widget = QLabel("Cannot get parameters", self)
                            table_widget.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
                    self.setCentralWidget(table_widget)
                    self.resize(650, 600)
                    break

    def centerize(self, prnt):
        self.__init()
        if prnt:
            self.move(
                prnt.window().frameGeometry().topLeft() + prnt.window().rect().center() - self.rect().center())

    def set_values(self):
        if self.inner_table_widget:
            if srv_ready(self.subs_and_clients.params_client):
                req = Params.Request()
                future = self.subs_and_clients.params_client.call_async(req)
                while rclpy.ok():
                    # TODO LOADING
                    if future.done():
                        try:
                            response = future.result()
                        except Exception as e:
                            QMessageBox.warning(self, "params_client", "Cannot get parameters\n"
                                                                       "Stacktrace: {}".format(e), QMessageBox.Ok)
                        else:
                            if response.names:
                                names = response.names[pass_args:]
                                values = response.values[pass_args:]
                                # n_rows = len(response.names)
                                n_rows = len(names)
                                for i in range(0, n_rows):
                                    v_item = self.inner_table_widget.cellWidget(i, 2)
                                    # v = response.values[i]
                                    v = values[i]
                                    v = '0' if v == '' else v
                                    if isinstance(v_item, QLabel):
                                        pass
                                    else:
                                        v_item.setValue(float(v))
                        break

    def apply_btn_clicked(self):
        if srv_ready(self.subs_and_clients.params_set_client):
            req = ParamsSet.Request()
            req.names = self.names
            values = []
            for i in range(0, self.inner_table_widget.rowCount()):
                values.append(str(self.inner_table_widget.cellWidget(i, 2).value()))
            req.values = values
            future = self.subs_and_clients.params_set_client.call_async(req)
            while rclpy.ok():
                # TODO LOADING
                if future.done():
                    try:
                        response = future.result()
                    except Exception as e:
                        QMessageBox.warning(self, "params_set_client", "Cannot change parameters.\n"
                                                                       "Stacktrace: {}".format(e), QMessageBox.Ok)
                    else:
                        if response.res:
                            self.close()
                        else:
                            QMessageBox.warning(self, "params_set_client", "Cannot change parameters.", QMessageBox.Ok)
                    break
