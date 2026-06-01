"""
WPILog (WPILOG 1.0) reader for AdvantageKit log files.

Parses .wpilog binary files into Python data structures for offline analysis.
Supports all primitive types, arrays, and WPILib struct types (Pose2d, Pose3d).

WPILOG format reference:
  https://github.com/wpilibsuite/allwpilib/blob/main/wpiutil/doc/wpilog.adoc
"""

from __future__ import annotations

import json
import math
import os
import struct
from collections import defaultdict
from functools import lru_cache
from typing import Any


# ---------------------------------------------------------------------------
# Low-level binary reader helpers
# ---------------------------------------------------------------------------

def _read_var(data: bytes, offset: int, n_bytes: int) -> int:
    """Read n_bytes little-endian unsigned integer from data at offset."""
    val = 0
    for i in range(n_bytes):
        val |= data[offset + i] << (8 * i)
    return val


def _read_string(data: bytes, offset: int) -> tuple[str, int]:
    """Read a length-prefixed string (uint32 len + bytes). Returns (string, new_offset)."""
    length = _read_var(data, offset, 4)
    offset += 4
    text = data[offset:offset + length].decode("utf-8", errors="replace")
    return text, offset + length


# ---------------------------------------------------------------------------
# Type decoders
# ---------------------------------------------------------------------------

def _decode_value(type_str: str, payload: bytes) -> Any:
    """Decode a raw payload bytes into a Python value based on the WPILog type string."""
    if type_str == "boolean":
        return bool(payload[0]) if payload else None

    if type_str == "int64":
        return struct.unpack_from("<q", payload)[0] if len(payload) >= 8 else None

    if type_str == "float":
        return struct.unpack_from("<f", payload)[0] if len(payload) >= 4 else None

    if type_str == "double":
        return struct.unpack_from("<d", payload)[0] if len(payload) >= 8 else None

    if type_str in ("string", "json"):
        length = _read_var(payload, 0, 4)
        return payload[4:4 + length].decode("utf-8", errors="replace")

    if type_str == "boolean[]":
        return [bool(b) for b in payload]

    if type_str == "int64[]":
        count = len(payload) // 8
        return list(struct.unpack_from(f"<{count}q", payload))

    if type_str == "float[]":
        count = len(payload) // 4
        return list(struct.unpack_from(f"<{count}f", payload))

    if type_str == "double[]":
        count = len(payload) // 8
        return list(struct.unpack_from(f"<{count}d", payload))

    if type_str == "string[]":
        results = []
        offset = 0
        count = _read_var(payload, offset, 4)
        offset += 4
        for _ in range(count):
            length = _read_var(payload, offset, 4)
            offset += 4
            results.append(payload[offset:offset + length].decode("utf-8", errors="replace"))
            offset += length
        return results

    if type_str == "struct:Pose2d[]" or type_str == "Pose2d[]":
        # Pose2d: x (double 8B), y (double 8B), rotation radians (double 8B) = 24 bytes each
        count = len(payload) // 24
        poses = []
        for i in range(count):
            x, y, rot = struct.unpack_from("<3d", payload, i * 24)
            poses.append({"x_m": round(x, 4), "y_m": round(y, 4), "yaw_deg": round(math.degrees(rot), 3)})
        return poses

    if type_str == "struct:Pose2d":
        if len(payload) >= 24:
            x, y, rot = struct.unpack_from("<3d", payload)
            return {"x_m": round(x, 4), "y_m": round(y, 4), "yaw_deg": round(math.degrees(rot), 3)}
        return None

    if type_str == "struct:Pose3d[]" or type_str == "Pose3d[]":
        # Pose3d: x,y,z (3 doubles) + qw,qx,qy,qz (4 doubles) = 56 bytes each
        count = len(payload) // 56
        poses = []
        for i in range(count):
            x, y, z, qw, qx, qy, qz = struct.unpack_from("<7d", payload, i * 56)
            yaw = _quat_to_yaw(qw, qx, qy, qz)
            poses.append({
                "x_m": round(x, 4), "y_m": round(y, 4), "z_m": round(z, 4),
                "yaw_deg": round(math.degrees(yaw), 3),
            })
        return poses

    if type_str == "struct:Pose3d":
        if len(payload) >= 56:
            x, y, z, qw, qx, qy, qz = struct.unpack_from("<7d", payload)
            yaw = _quat_to_yaw(qw, qx, qy, qz)
            return {"x_m": round(x, 4), "y_m": round(y, 4), "z_m": round(z, 4), "yaw_deg": round(math.degrees(yaw), 3)}
        return None

    if type_str == "struct:SwerveModuleState[]":
        # SwerveModuleState: angle (double radians) + speed (double m/s) = 16 bytes each
        count = len(payload) // 16
        states = []
        for i in range(count):
            angle, speed = struct.unpack_from("<2d", payload, i * 16)
            states.append({"angle_deg": round(math.degrees(angle), 2), "speed_mps": round(speed, 4)})
        return states

    if type_str == "struct:ChassisSpeeds":
        # vx, vy, omega (3 doubles) = 24 bytes
        if len(payload) >= 24:
            vx, vy, omega = struct.unpack_from("<3d", payload)
            return {"vx_mps": round(vx, 4), "vy_mps": round(vy, 4), "omega_radps": round(omega, 4)}
        return None

    # Unknown type — return hex for small payloads, length for large
    if len(payload) <= 32:
        return f"<{type_str}: 0x{payload.hex()}>"
    return f"<{type_str}: {len(payload)} bytes>"


def _quat_to_yaw(qw: float, qx: float, qy: float, qz: float) -> float:
    """Convert WPILib quaternion (w,x,y,z) to yaw in radians."""
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


# ---------------------------------------------------------------------------
# WPILOG parser
# ---------------------------------------------------------------------------

class WPILogRecord:
    __slots__ = ("entry_id", "timestamp_us", "payload")

    def __init__(self, entry_id: int, timestamp_us: int, payload: bytes):
        self.entry_id = entry_id
        self.timestamp_us = timestamp_us
        self.payload = payload


class WPILogChannel:
    __slots__ = ("name", "type_str", "metadata", "samples")

    def __init__(self, name: str, type_str: str, metadata: str):
        self.name = name
        self.type_str = type_str
        self.metadata = metadata
        # List of (timestamp_s: float, value: Any)
        self.samples: list[tuple[float, Any]] = []


class WPILog:
    """Parsed WPILOG file."""

    MAGIC = b"WPILOG"

    def __init__(self, path: str):
        self.path = path
        self.channels: dict[str, WPILogChannel] = {}  # name → channel
        self._id_map: dict[int, WPILogChannel] = {}    # entry_id → channel
        self.extra_header: dict = {}
        self.duration_s: float = 0.0
        self._parse()

    def _parse(self) -> None:
        with open(self.path, "rb") as f:
            data = f.read()

        # Validate header
        if not data.startswith(self.MAGIC):
            raise ValueError(f"Not a valid WPILOG file: {self.path}")

        offset = len(self.MAGIC)
        # Version (uint16 LE)
        version = _read_var(data, offset, 2)
        offset += 2
        if version >> 8 != 1:
            raise ValueError(f"Unsupported WPILOG version 0x{version:04x}")

        # Extra header (uint32 length + bytes)
        extra_len = _read_var(data, offset, 4)
        offset += 4
        if extra_len > 0:
            try:
                self.extra_header = json.loads(data[offset:offset + extra_len].decode("utf-8", errors="replace"))
            except json.JSONDecodeError:
                self.extra_header = {}
        offset += extra_len

        max_timestamp_us = 0

        # Parse records
        while offset < len(data):
            if offset + 1 > len(data):
                break

            bitfield = data[offset]
            offset += 1

            entry_id_len = (bitfield & 0x03) + 1        # bits 0-1
            size_len = ((bitfield >> 2) & 0x03) + 1     # bits 2-3
            timestamp_len = ((bitfield >> 4) & 0x07) + 1  # bits 4-6

            if offset + entry_id_len + size_len + timestamp_len > len(data):
                break

            entry_id = _read_var(data, offset, entry_id_len)
            offset += entry_id_len

            payload_size = _read_var(data, offset, size_len)
            offset += size_len

            timestamp_us = _read_var(data, offset, timestamp_len)
            offset += timestamp_len

            if offset + payload_size > len(data):
                break

            payload = data[offset:offset + payload_size]
            offset += payload_size

            if timestamp_us > max_timestamp_us:
                max_timestamp_us = timestamp_us

            # Dispatch
            if entry_id == 0:
                self._handle_control(payload)
            else:
                ch = self._id_map.get(entry_id)
                if ch is not None:
                    try:
                        value = _decode_value(ch.type_str, payload)
                        ch.samples.append((timestamp_us / 1_000_000.0, value))
                    except Exception:
                        pass  # Skip malformed samples

        self.duration_s = max_timestamp_us / 1_000_000.0

    def _handle_control(self, payload: bytes) -> None:
        if not payload:
            return
        ctrl_type = payload[0]

        if ctrl_type == 0:  # Start
            if len(payload) < 5:
                return
            entry_id = _read_var(payload, 1, 4)
            name, pos = _read_string(payload, 5)
            type_str, pos = _read_string(payload, pos)
            metadata, _ = _read_string(payload, pos)
            ch = WPILogChannel(name, type_str, metadata)
            self.channels[name] = ch
            self._id_map[entry_id] = ch

        elif ctrl_type == 1:  # Finish
            if len(payload) >= 5:
                entry_id = _read_var(payload, 1, 4)
                self._id_map.pop(entry_id, None)

        elif ctrl_type == 2:  # SetMetadata
            if len(payload) >= 5:
                entry_id = _read_var(payload, 1, 4)
                metadata, _ = _read_string(payload, 5)
                ch = self._id_map.get(entry_id)
                if ch:
                    ch.metadata = metadata


# ---------------------------------------------------------------------------
# Cache: avoid re-parsing the same file
# ---------------------------------------------------------------------------

_log_cache: dict[str, WPILog] = {}


def load_log(path: str) -> WPILog:
    """Load (and cache) a WPILOG file. Raises on error."""
    abs_path = os.path.abspath(path)
    if abs_path not in _log_cache:
        _log_cache[abs_path] = WPILog(abs_path)
    return _log_cache[abs_path]


def evict_log(path: str) -> None:
    """Remove a log from the cache (call after the file is replaced)."""
    abs_path = os.path.abspath(path)
    _log_cache.pop(abs_path, None)


# ---------------------------------------------------------------------------
# Analysis helpers
# ---------------------------------------------------------------------------

def resample_signal(
    samples: list[tuple[float, Any]],
    start_s: float | None,
    end_s: float | None,
    max_samples: int,
) -> list[tuple[float, Any]]:
    """Slice and optionally downsample a list of (time_s, value) tuples."""
    filtered = [
        s for s in samples
        if (start_s is None or s[0] >= start_s) and (end_s is None or s[0] <= end_s)
    ]
    if len(filtered) <= max_samples:
        return filtered
    step = len(filtered) / max_samples
    return [filtered[int(i * step)] for i in range(max_samples)]


def pose2d_drift_stats(
    actual: list[tuple[float, dict]],
    estimated: list[tuple[float, dict]],
) -> dict:
    """
    Compute drift statistics between actual and estimated Pose2d time series.

    Interpolates estimated to actual timestamps. Returns mean/max/p95 drift in meters.
    """
    if not actual or not estimated:
        return {"error": "One or both pose series is empty"}

    # Build lookup: nearest estimated sample for each actual timestamp
    est_times = [s[0] for s in estimated]
    est_poses = [s[1] for s in estimated]

    errors_m: list[float] = []
    heading_errors_deg: list[float] = []

    for t_act, p_act in actual:
        if not isinstance(p_act, list) or not p_act:
            continue
        p_act0 = p_act[0] if isinstance(p_act, list) else p_act

        # Find nearest estimated sample by timestamp
        idx = _nearest_index(est_times, t_act)
        p_est_raw = est_poses[idx]
        p_est0 = p_est_raw[0] if isinstance(p_est_raw, list) else p_est_raw
        if not isinstance(p_est0, dict):
            continue

        dx = p_act0.get("x_m", 0.0) - p_est0.get("x_m", 0.0)
        dy = p_act0.get("y_m", 0.0) - p_est0.get("y_m", 0.0)
        errors_m.append(math.sqrt(dx * dx + dy * dy))

        dh = abs(p_act0.get("yaw_deg", 0.0) - p_est0.get("yaw_deg", 0.0))
        if dh > 180.0:
            dh = 360.0 - dh
        heading_errors_deg.append(dh)

    if not errors_m:
        return {"error": "No valid overlapping pose samples found"}

    errors_m.sort()
    heading_errors_deg.sort()
    n = len(errors_m)

    return {
        "sample_count": n,
        "translation_drift_mean_m": round(sum(errors_m) / n, 4),
        "translation_drift_max_m": round(errors_m[-1], 4),
        "translation_drift_p95_m": round(errors_m[int(0.95 * n)], 4),
        "heading_error_mean_deg": round(sum(heading_errors_deg) / n, 3),
        "heading_error_max_deg": round(heading_errors_deg[-1], 3),
    }


def _nearest_index(sorted_times: list[float], t: float) -> int:
    """Binary search for the index of the nearest timestamp."""
    lo, hi = 0, len(sorted_times) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if sorted_times[mid] < t:
            lo = mid + 1
        else:
            hi = mid
    return lo
