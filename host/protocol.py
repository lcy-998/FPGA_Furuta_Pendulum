"""Wire protocol shared with src/uart_control_link.v."""
from __future__ import annotations

import struct
from dataclasses import dataclass

SYNC = 0xA5
TYPE_TELEMETRY = 0x01
TYPE_SET_GAINS = 0x10
TYPE_IDENT = 0x12
TYPE_ACK = 0x81
FIELDS = ("theta", "dtheta", "alpha", "dalpha", "theta_ref", "dtheta_ref", "alpha_ref", "dalpha_ref", "pwm_applied", "pwm_exc", "ident_active")
IDENTIFICATION_FIELDS = ("time_s", "telemetry_seq", "theta", "dtheta", "alpha", "alpha_ref", "dalpha", "pwm_applied", "pwm_exc")
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

def identification_command(sequence: int, enabled: bool) -> bytes:
    return frame(TYPE_IDENT, sequence, bytes((1 if enabled else 0,)))


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


def telemetry_values(payload: bytes) -> dict[str, float]:
    if len(payload) != 44:
        raise ValueError(f"遥测长度错误：{len(payload)}")
    values = {name: raw / 65536.0 for name, raw in zip(FIELDS[:8], struct.unpack("<8i", payload[:32]))}
    values["pwm_applied"] = float(struct.unpack("<i", payload[32:36])[0])
    values["pwm_exc"] = float(struct.unpack("<i", payload[36:40])[0])
    values["ident_active"] = float(struct.unpack("<I", payload[40:44])[0])
    return values
