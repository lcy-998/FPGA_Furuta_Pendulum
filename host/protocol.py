"""Wire protocol shared with src/uart_control_link.v."""
from __future__ import annotations

import struct
from dataclasses import dataclass

SYNC = 0xA5
TYPE_TELEMETRY = 0x01
TYPE_SET_GAINS = 0x10
TYPE_SET_DEADBAND = 0x11
TYPE_ACK = 0x81
FIELDS = ("theta", "dtheta", "alpha", "dalpha", "theta_ref", "dtheta_ref", "alpha_ref", "dalpha_ref")
TELEMETRY_FIELDS = FIELDS + ("pwm_duty",)
GAIN_DEFAULTS = (-40059 / 1024, -696 / 1024, -9062 / 1024, -3994 / 1024, 0.0)


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame(frame_type: int, sequence: int, payload: bytes) -> bytes:
    header = bytes((frame_type, sequence & 0xFF, len(payload)))
    body = header + payload
    return bytes((SYNC,)) + body + struct.pack("<H", crc16(body))


def gain_command(sequence: int, gains: list[float]) -> bytes:
    if len(gains) != 5:
        raise ValueError("必须提供 K1 到 K5 五个增益")
    raw = [round(value * 1024) for value in gains]
    if any(value < -(1 << 17) or value >= (1 << 17) for value in raw):
        raise ValueError("增益必须在有符号 18 位 Q10 范围内（约 -128 到 127.999）")
    return frame(TYPE_SET_GAINS, sequence, struct.pack("<5i", *raw))


@dataclass
class Packet:
    frame_type: int
    sequence: int
    payload: bytes


class Parser:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list[Packet]:
        self.buffer.extend(data)
        packets: list[Packet] = []
        while True:
            try:
                start = self.buffer.index(SYNC)
            except ValueError:
                self.buffer.clear()
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 6:
                break
            length = self.buffer[3]
            if length > 64:
                del self.buffer[0]
                continue
            total = 1 + 3 + length + 2
            if len(self.buffer) < total:
                break
            body = bytes(self.buffer[1:4 + length])
            received = int.from_bytes(self.buffer[4 + length:total], "little")
            if crc16(body) == received:
                packets.append(Packet(self.buffer[1], self.buffer[2], bytes(self.buffer[4:4 + length])))
                del self.buffer[:total]
            else:
                del self.buffer[0]
        return packets


def deadband_command(sequence: int, value: int) -> bytes:
    if not 0 <= value <= 1000:
        raise ValueError("??????? 0 ? 1000 ??")
    return frame(TYPE_SET_DEADBAND, sequence, struct.pack("<H", value))


def telemetry_values(payload: bytes) -> dict[str, float]:
    if len(payload) not in (32, 34):
        raise ValueError(f"遥测长度错误：{len(payload)}")
    values = {name: raw / 65536.0 for name, raw in zip(FIELDS, struct.unpack("<8i", payload[:32]))}
    if len(payload) == 34:
        values["pwm_duty"] = struct.unpack("<h", payload[32:34])[0]
    return values
