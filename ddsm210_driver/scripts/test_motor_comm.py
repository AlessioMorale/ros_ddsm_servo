#!/usr/bin/env python3
"""Standalone DDSM210 bus diagnostic tool (no ROS dependency).

Uses the "Obtain Mode Feedback" (0x75) command, which is a pure read with no motion,
to probe communication with one or more motor IDs at a time and report success rates.
This lets us isolate whether communication problems are per-motor (wiring/ID) or only
appear when multiple motors are addressed back-to-back (bus contention).

Usage:
  test_motor_comm.py /dev/ttyAMA0 [--baud 115200] [--count 50] [--delay-ms 0] \
      [--ids 1,2,3,4]
"""
import argparse
import itertools
import sys
import time

import serial

CRC8_TABLE = [
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E, 0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0, 0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D, 0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5, 0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58, 0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6, 0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B, 0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F, 0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92, 0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C, 0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1, 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49, 0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4, 0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A, 0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7, 0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
]

CMD_GET_MODE = 0x75
CMD_DRIVE = 0x64
CMD_MODE_SWITCH = 0xA0
MODE_VELOCITY = 0x02
PACKET_SIZE = 10


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc = CRC8_TABLE[crc ^ byte]
    return crc


def build_packet(motor_id: int, cmd: int, payload: bytes = b"\x00" * 7) -> bytes:
    assert len(payload) == 7
    body = bytes([motor_id, cmd]) + payload
    return body + bytes([crc8(body)])


def build_get_mode_packet(motor_id: int) -> bytes:
    return build_packet(motor_id, CMD_GET_MODE)


def build_mode_switch_packet(motor_id: int, mode: int) -> bytes:
    return build_packet(motor_id, CMD_MODE_SWITCH, bytes([mode]) + b"\x00" * 6)


def build_drive_packet(motor_id: int, target: int = 0) -> bytes:
    # target=0 is safe (no motion); speed is a big-endian signed 16-bit, unit 0.1rpm.
    speed = target.to_bytes(2, byteorder="big", signed=True)
    return build_packet(motor_id, CMD_DRIVE, speed + b"\x00" * 5)


def query(ser: serial.Serial, motor_id: int, packet: bytes, read_timeout: float):
    """Sends a single request and reads back whatever arrives. Returns
    (ok, raw_bytes) where ok is True only for a full, checksum-valid response
    addressed to motor_id."""
    ser.reset_input_buffer()
    ser.write(packet)
    ser.flush()

    deadline = time.monotonic() + read_timeout
    buf = b""
    while time.monotonic() < deadline and len(buf) < PACKET_SIZE:
        chunk = ser.read(PACKET_SIZE - len(buf))
        if chunk:
            buf += chunk

    if len(buf) != PACKET_SIZE:
        return False, buf
    if crc8(buf[:-1]) != buf[-1]:
        return False, buf
    if buf[0] != motor_id:
        return False, buf
    return True, buf


def query_mode(ser: serial.Serial, motor_id: int, read_timeout: float):
    return query(ser, motor_id, build_get_mode_packet(motor_id), read_timeout)


def query_drive(ser: serial.Serial, motor_id: int, read_timeout: float):
    return query(ser, motor_id, build_drive_packet(motor_id, 0), read_timeout)


def run_single_motor_tests(ser, ids, count, delay_ms, read_timeout, query_fn):
    print("\n=== Single motor tests ===")
    for motor_id in ids:
        ok_count = 0
        raws = []
        for _ in range(count):
            ok, raw = query_fn(ser, motor_id, read_timeout)
            ok_count += ok
            if not ok:
                raws.append(raw)
            if delay_ms:
                time.sleep(delay_ms / 1000.0)
        print(f"motor {motor_id}: {ok_count}/{count} ok", end="")
        if raws:
            sample = raws[0].hex()
            print(f"  (sample failed read: {sample!r}, {len(raws)} failures total)")
        else:
            print()


def run_pairwise_tests(ser, ids, count, delay_ms, read_timeout, query_fn):
    print("\n=== Pairwise (alternating) tests ===")
    for a, b in itertools.combinations(ids, 2):
        ok_a = ok_b = 0
        for _ in range(count):
            ok, _ = query_fn(ser, a, read_timeout)
            ok_a += ok
            if delay_ms:
                time.sleep(delay_ms / 1000.0)
            ok, _ = query_fn(ser, b, read_timeout)
            ok_b += ok
            if delay_ms:
                time.sleep(delay_ms / 1000.0)
        print(f"motor {a} vs motor {b}: {a}={ok_a}/{count} ok, {b}={ok_b}/{count} ok")


def run_round_robin_test(ser, ids, count, delay_ms, read_timeout, query_fn):
    print("\n=== Round-robin (all motors, driver-like order) tests ===")
    ok_counts = {motor_id: 0 for motor_id in ids}
    for _ in range(count):
        for motor_id in ids:
            ok, _ = query_fn(ser, motor_id, read_timeout)
            ok_counts[motor_id] += ok
            if delay_ms:
                time.sleep(delay_ms / 1000.0)
    for motor_id in ids:
        print(f"motor {motor_id}: {ok_counts[motor_id]}/{count} ok")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="Serial device, e.g. /dev/ttyAMA0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--count", type=int, default=50, help="Queries per test")
    parser.add_argument("--delay-ms", type=float, default=0.0, help="Delay between queries")
    parser.add_argument("--read-timeout", type=float, default=0.05, help="Per-query read timeout (s)")
    parser.add_argument("--ids", default="1,2,3,4", help="Comma-separated motor ids to test")
    parser.add_argument(
        "--cmd", choices=["get-mode", "drive"], default="get-mode",
        help="Which command to probe with: get-mode (0x75, pure read) or drive (0x64, target=0)")
    args = parser.parse_args()

    ids = [int(x) for x in args.ids.split(",") if x.strip()]
    query_fn = query_drive if args.cmd == "drive" else query_mode

    with serial.Serial(args.port, args.baud, timeout=0) as ser:
        if args.cmd == "drive":
            # Drive commands are only accepted in velocity loop mode.
            for motor_id in ids:
                ser.reset_input_buffer()
                ser.write(build_mode_switch_packet(motor_id, MODE_VELOCITY))
                ser.flush()
                time.sleep(0.01)
                ser.read(PACKET_SIZE)

        run_single_motor_tests(ser, ids, args.count, args.delay_ms, args.read_timeout, query_fn)
        run_pairwise_tests(ser, ids, args.count, args.delay_ms, args.read_timeout, query_fn)
        run_round_robin_test(ser, ids, args.count, args.delay_ms, args.read_timeout, query_fn)


if __name__ == "__main__":
    sys.exit(main())
