from contextlib import closing
from datetime import datetime
from socket import socket, AF_INET, SOCK_DGRAM, gaierror
import struct
import ntplib
from time import ctime, gmtime, strftime

NTP_PACKET_FORMAT = "!12I"
NTP_DELTA = 2208988800  # 1970-01-01 00:00:00
# If you wonder what '\x1b' + 47 * '\0' stands for: https://stackoverflow.com/a/26938508/1422096
NTP_QUERY = '\x1b' + 47 * '\0'

__ntp_client = ntplib.NTPClient()


def ntp_time(host="pool.ntp.org", port=123):
    # noinspection PyBroadException
    try:
        with closing(socket(AF_INET, SOCK_DGRAM)) as s:
            s.sendto(NTP_QUERY.encode(), (host, port))
            msg, address = s.recvfrom(1024)
        unpacked = struct.unpack(NTP_PACKET_FORMAT,
                                 msg[0:struct.calcsize(NTP_PACKET_FORMAT)])
        return datetime.utcfromtimestamp(unpacked[10] + float(unpacked[11]) / 2 ** 32 - NTP_DELTA)
    except Exception:
        return None


def ntplib_ntp_time(host="pool.ntp.org"):
    # noinspection PyBroadException
    try:
        response = __ntp_client.request(host)
        return datetime.utcfromtimestamp(response.tx_time)
    except Exception:
        return None
