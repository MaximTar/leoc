import rclpy
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QMainWindow, QLabel, QSpinBox, QTableWidget, QTableWidgetItem, QDoubleSpinBox, \
    QPushButton, QWidget, QVBoxLayout, QMessageBox
from antenna_interfaces.srv import Params, ParamsInfo, ParamsSet


# noinspection PyUnresolvedReferences
class ParametersWindow(QMainWindow):
    def __init__(self, subs_and_clients, parent=None):
        super().__init__(parent=parent)
        self.setWindowTitle("Parameters")
        self.setWindowModality(Qt.ApplicationModal)
        self.subs_and_clients = subs_and_clients

    def __init(self):
        req = ParamsInfo.Request()
        while not self.subs_and_clients.params_info_client.wait_for_service(timeout_sec=1.0):
            # TODO LOADING
            print('Service params_info_client is not available, waiting...')

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
                        self.names = response.names
                        table_widget = QWidget(self)
                        table_layout = QVBoxLayout()
                        self.inner_table_widget = QTableWidget(self)
                        self.inner_table_widget.setColumnCount(4)
                        n_rows = len(response.names)
                        self.inner_table_widget.setRowCount(n_rows)
                        self.inner_table_widget.setHorizontalHeaderLabels(["Name", "Type", "Value", "Description"])
                        for i in range(0, n_rows):
                            n_item = QTableWidgetItem(response.names[i])
                            t_item = QTableWidgetItem(response.types[i])
                            if response.types[i] == 'uint8':
                                v_item = QSpinBox(self)
                                v_item.setMaximum(255)
                            elif response.types[i] == 'uint16':
                                v_item = QSpinBox(self)
                                v_item.setMaximum(65535)
                            elif response.types[i] == 'float':
                                v_item = QDoubleSpinBox(self)
                                v_item.setMaximum(360)
                            else:
                                v_item = QLabel("Call admin")
                            d_item = QTableWidgetItem(response.descs[i])
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
            req = Params.Request()
            while not self.subs_and_clients.params_client.wait_for_service(timeout_sec=1.0):
                # TODO LOADING
                print('Service params_client is not available, waiting...')

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
                            n_rows = len(response.names)
                            for i in range(0, n_rows):
                                v_item = self.inner_table_widget.cellWidget(i, 2)
                                v = response.values[i]
                                v = '0' if v == '' else v
                                v_item.setValue(float(v))
                    break

    def apply_btn_clicked(self):
        req = ParamsSet.Request()
        req.names = self.names
        values = []
        for i in range(0, self.inner_table_widget.rowCount()):
            values.append(str(self.inner_table_widget.cellWidget(i, 2).value()))
        req.values = values
        while not self.subs_and_clients.params_set_client.wait_for_service(timeout_sec=1.0):
            # TODO LOADING
            print('Service params_set_client is not available, waiting...')
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
                        pass
                    else:
                        QMessageBox.warning(self, "params_set_client", "Cannot change parameters.", QMessageBox.Ok)
                break
