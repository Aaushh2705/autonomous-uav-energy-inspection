#!/usr/bin/env python3
"""Provide a minimal MAVLink GCS heartbeat for local PX4 SITL."""

import socket
import struct
import time


PX4_GCS_PORT = 14550
MAVLINK_STX_V1 = 0xFE
MAVLINK_MSG_ID_HEARTBEAT = 0
MAVLINK_HEARTBEAT_CRC_EXTRA = 50
MAV_TYPE_GCS = 6
MAV_AUTOPILOT_INVALID = 8
MAV_STATE_ACTIVE = 4


def x25_crc(data: bytes) -> int:
    """Calculate the checksum used by MAVLink 1 frames."""
    crc = 0xFFFF
    for value in data:
        temporary = value ^ (crc & 0xFF)
        temporary ^= (temporary << 4) & 0xFF
        crc = (
            (crc >> 8)
            ^ (temporary << 8)
            ^ (temporary << 3)
            ^ (temporary >> 4)
        ) & 0xFFFF
    return crc


def heartbeat_frame(sequence: int) -> bytes:
    """Build a MAVLink 1 GCS heartbeat frame."""
    payload = struct.pack(
        '<IBBBBB',
        0,
        MAV_TYPE_GCS,
        MAV_AUTOPILOT_INVALID,
        0,
        MAV_STATE_ACTIVE,
        3,
    )
    header_without_magic = bytes([
        len(payload),
        sequence,
        255,
        190,
        MAVLINK_MSG_ID_HEARTBEAT,
    ])
    checksum = x25_crc(
        header_without_magic + payload + bytes([MAVLINK_HEARTBEAT_CRC_EXTRA]))
    return (
        bytes([MAVLINK_STX_V1])
        + header_without_magic
        + payload
        + struct.pack('<H', checksum)
    )


def main() -> None:
    """Listen for local PX4 MAVLink traffic and reply with GCS heartbeats."""
    connection = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    connection.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    connection.bind(('127.0.0.1', PX4_GCS_PORT))
    connection.settimeout(0.5)

    print('SITL GCS heartbeat waiting for PX4 on UDP 14550...', flush=True)
    peer = None
    sequence = 0
    next_heartbeat = 0.0

    try:
        while True:
            try:
                _, sender = connection.recvfrom(4096)
                if peer != sender:
                    peer = sender
                    print(f'PX4 detected at {peer[0]}:{peer[1]}', flush=True)
            except socket.timeout:
                pass

            now = time.monotonic()
            if peer is not None and now >= next_heartbeat:
                connection.sendto(heartbeat_frame(sequence), peer)
                sequence = (sequence + 1) % 256
                next_heartbeat = now + 1.0
    except KeyboardInterrupt:
        print('SITL GCS heartbeat stopped.', flush=True)
    finally:
        connection.close()


if __name__ == '__main__':
    main()
