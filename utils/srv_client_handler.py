from PyQt5.QtWidgets import QMessageBox, QPushButton


def srv_ready(client):
    if not client.wait_for_service(timeout_sec=1.0):
        msg_box = QMessageBox()
        msg_box.setIcon(QMessageBox.Critical)
        msg_box.setText("{} is not responding".format(client.srv_name))
        msg_box.setWindowTitle(client.srv_name)
        ta_btn = QPushButton("Try again")
        ca_btn = QPushButton("Cancel")
        msg_box.addButton(ta_btn, QMessageBox.ActionRole)
        msg_box.addButton(ca_btn, QMessageBox.ActionRole)
        msg_box.exec()
        if msg_box.clickedButton() == ta_btn:
            srv_ready(client)
        elif msg_box.clickedButton() == ca_btn:
            return False
    else:
        return True
