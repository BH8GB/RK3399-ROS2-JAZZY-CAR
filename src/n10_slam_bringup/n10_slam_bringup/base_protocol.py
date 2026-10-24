from dataclasses import dataclass
import struct
from typing import Iterable


HEADER = 0xAA
CMD_WHEEL_RPM = 0x10
CMD_STOP = 0x22
CMD_TELEMETRY = 0x35
MAX_PAYLOAD_SIZE = 64

STATUS_IMU_VALID = 0x0001
STATUS_COMMAND_ACTIVE = 0x0002
STATUS_WATCHDOG_STOPPED = 0x0004
STATUS_UART_RECOVERED = 0x0008
STATUS_STALL_PROTECTED = 0x0010

_WHEEL_RPM = struct.Struct('<4h')
_TELEMETRY = struct.Struct('<I4i4h2h2H')


@dataclass(frozen=True)
class Frame:
    command: int
    payload: bytes


@dataclass(frozen=True)
class Telemetry:
    timestamp_ms: int
    encoder_counts: tuple[int, int, int, int]
    wheel_rpm: tuple[int, int, int, int]
    yaw_degrees: float
    gyro_rate_dps: float
    flags: int
    stall_events: int


def encode_frame(command: int, payload: bytes = b'') -> bytes:
    if not 0 <= command <= 0xFF:
        raise ValueError('command must fit in one byte')
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise ValueError('payload is too large')
    checksum = (command + len(payload) + sum(payload)) & 0xFF
    return bytes((HEADER, command, len(payload))) + payload + bytes((checksum,))


def encode_wheel_rpm(values: Iterable[int]) -> bytes:
    rpm = tuple(int(round(value)) for value in values)
    if len(rpm) != 4:
        raise ValueError('exactly four wheel RPM values are required')
    return encode_frame(CMD_WHEEL_RPM, _WHEEL_RPM.pack(*rpm))


def encode_stop() -> bytes:
    return encode_frame(CMD_STOP)


def decode_telemetry(payload: bytes) -> Telemetry:
    if len(payload) != _TELEMETRY.size:
        raise ValueError(
            f'telemetry payload must be {_TELEMETRY.size} bytes, got {len(payload)}'
        )
    values = _TELEMETRY.unpack(payload)
    return Telemetry(
        timestamp_ms=values[0],
        encoder_counts=tuple(values[1:5]),
        wheel_rpm=tuple(values[5:9]),
        yaw_degrees=values[9] / 10.0,
        gyro_rate_dps=values[10] / 10.0,
        flags=values[11],
        stall_events=values[12],
    )


class FrameParser:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def reset(self) -> None:
        self._buffer.clear()

    def feed(self, data: bytes) -> list[Frame]:
        self._buffer.extend(data)
        frames: list[Frame] = []

        while True:
            header_index = self._buffer.find(bytes((HEADER,)))
            if header_index < 0:
                self._buffer.clear()
                break
            if header_index:
                del self._buffer[:header_index]
            if len(self._buffer) < 4:
                break

            command = self._buffer[1]
            payload_size = self._buffer[2]
            if payload_size > MAX_PAYLOAD_SIZE:
                del self._buffer[0]
                continue

            frame_size = payload_size + 4
            if len(self._buffer) < frame_size:
                break
            candidate = self._buffer[:frame_size]
            expected = (command + payload_size + sum(candidate[3:-1])) & 0xFF
            if candidate[-1] != expected:
                del self._buffer[0]
                continue

            frames.append(Frame(command=command, payload=bytes(candidate[3:-1])))
            del self._buffer[:frame_size]

        return frames
