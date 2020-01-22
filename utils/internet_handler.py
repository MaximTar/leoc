import socket
import platform  # For getting the operating system name
import subprocess  # For executing a shell command


# slightly altered code from https://stackoverflow.com/questions/2953462/pinging-servers-in-python
def ping(host):
    """
    Returns True if host responds to a ping request.
    """
    # noinspection PyBroadException
    try:
        ip = socket.gethostbyname(host)
    except Exception:
        return False
    # Option for the number of packets as a function of
    param = '-n' if platform.system().lower() == 'windows' else '-c'

    # Building the command. Ex: "ping -c 1 google.com"
    command = ['ping', param, '1', ip]

    return subprocess.call(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0
