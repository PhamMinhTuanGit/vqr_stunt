"""Temporary test tool -- delete after debugging."""
import socket
import struct
import time

HOST = "127.0.0.1"
PORT = 12121
BUTTON_INDEX = {"A": 6, "B": 7, "Y": 9}


def build_packet(seq, buttons_pressed=(), left_axis_y=0):
    buttons = [0] * 10
    for name in buttons_pressed:
        buttons[BUTTON_INDEX[name]] = 1
    data = struct.pack("<10H4h2H", *buttons, 0, left_axis_y, 0, 0, 0, 0)
    crc16 = sum(data) & 0xFFFF
    return struct.pack("<2sBHHBH32s", b"\x55\x66", 0, len(data), seq & 0xFFFF, 1, crc16, data)


def send_for(sock, seconds, label, **kwargs):
    print(f"--- {label} for {seconds}s ---", flush=True)
    seq = 0
    t0 = time.time()
    while time.time() - t0 < seconds:
        sock.sendto(build_packet(seq, **kwargs), (HOST, PORT))
        seq += 1
        time.sleep(0.02)


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    send_for(sock, 0.3, "press Y (StandUp)", buttons_pressed=("Y",))
    send_for(sock, 3.0, "wait for stand", buttons_pressed=())
    send_for(sock, 0.3, "press A (RLControl)", buttons_pressed=("A",))
    send_for(sock, 1.5, "settle into RL control", buttons_pressed=())
    send_for(sock, 0.3, "press B (LieDown)", buttons_pressed=("B",))
    send_for(sock, 2.0, "observe Phase 1", buttons_pressed=())
    print("done", flush=True)


if __name__ == "__main__":
    main()
