import socket
import ctypes
import json
import logging
import pytest
import sys
import os

logger = logging.getLogger(__name__)

MAC_ADDRESS = "12:00:3B:AF:21:15"
PSM_VALUE = 0x0080

AF_BLUETOOTH = 31
BTPROTO_L2CAP = 0
SOCK_SEQPACKET = 5
BDADDR_LE_PUBLIC = 1
BDADDR_LE_RANDOM = 2


class sockaddr_l2(ctypes.Structure):

    _fields_ = [
        ("l2_family", ctypes.c_ushort),
        ("l2_psm", ctypes.c_ushort),
        ("l2_bdaddr", ctypes.c_ubyte * 6),
        ("l2_cid", ctypes.c_ushort),
        ("l2_bdaddr_type", ctypes.c_ubyte),
    ]


class CtypesBleClient:
    def __init__(self, mac, psm, timeout=5.0):
        self.mac = mac
        self.psm = psm
        self.timeout = timeout
        self.sock = None

    def connect(self):
        self.sock = socket.socket(AF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP)
        # Видаляємо settimeout звідси, щоб сокет був БЛОКУЮЧИМ під час connect()

        addr = sockaddr_l2()
        addr.l2_family = AF_BLUETOOTH
        addr.l2_psm = (
            self.psm
            if sys.byteorder == "little"
            else ((self.psm & 0xFF) << 8) | (self.psm >> 8)
        )

        mac_bytes = [int(x, 16) for x in reversed(self.mac.split(":"))]
        for i, b in enumerate(mac_bytes):
            addr.l2_bdaddr[i] = b

        addr.l2_cid = 0
        addr.l2_bdaddr_type = BDADDR_LE_PUBLIC

        libc = ctypes.CDLL("libc.so.6", use_errno=True)

        # Виклик заблокує виконання, поки пристрій не підключиться (або ОС не викине таймаут)
        res = libc.connect(self.sock.fileno(), ctypes.byref(addr), ctypes.sizeof(addr))

        if res < 0:
            errno_val = ctypes.get_errno()
            raise OSError(errno_val, os.strerror(errno_val))

        # ПІСЛЯ успішного підключення встановлюємо таймаут для читання та запису даних
        self.sock.settimeout(self.timeout)

    def send_request(self, req: dict) -> dict:
        payload = json.dumps(req).encode("utf-8")
        self.sock.sendall(payload)

        resp_bytes = self.sock.recv(1024)
        return json.loads(resp_bytes.decode("utf-8"))

    def close(self):
        if self.sock:
            self.sock.close()


@pytest.fixture(scope="module")
def ble_client():
    client = CtypesBleClient(MAC_ADDRESS, PSM_VALUE)
    client.connect()
    yield client
    client.close()


@pytest.mark.parametrize(
    "name, request_data, expected_status",
    [
        ("get_vin", {"id": 1, "cmd": "get_vin"}, "OK"),
        ("get_live_data", {"id": 2, "cmd": {"get_live_data": {"pid": 12}}}, "OK"),
        ("get_stored_dtcs", {"id": 3, "cmd": "get_stored_dtcs"}, "OK"),
    ],
)
def test_obd_commands(ble_client, name, request_data, expected_status):
    logger.info(f"Test TX: {request_data}")
    response = ble_client.send_request(request_data)
    logger.info(f"Test RX: {response}")

    assert response.get("status") == expected_status
    assert response.get("id") == request_data["id"]
