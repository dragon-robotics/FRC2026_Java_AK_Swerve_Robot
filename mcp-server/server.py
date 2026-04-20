"""
FRC Telemetry MCP Server
========================
Connects to a WPILib sim's NetworkTables 4 server and exposes robot telemetry
to VS Code Copilot via MCP tools.

Architecture:
  WPILib Sim (Java) → NT4 (localhost:5810) → pyntcore → MCP Server (stdio) → Copilot

Usage:
  python -m server          # Run directly (stdio transport for VS Code)
  python server.py          # Same thing
"""

from __future__ import annotations

import json
import sys
import time
import threading
from collections import defaultdict, deque
from typing import Any

import ntcore
from mcp.server.fastmcp import FastMCP

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

NT_SERVER_HOST = "localhost"
NT_SERVER_PORT = 5810
RING_BUFFER_SECONDS = 10.0
RING_BUFFER_MAX_SAMPLES = 500  # ~10s at 50Hz

# AdvantageKit publishes under these prefixes
AK_OUTPUTS_PREFIX = "/AdvantageKit/RealOutputs/"
AK_INPUTS_PREFIX = "/AdvantageKit/"

# Subsystem → NT key prefixes (short-form, without the AK prefix)
SUBSYSTEM_PREFIXES: dict[str, list[str]] = {
    "Drive": [
        "SwerveStates/",
        "SwerveChassisSpeeds/",
        "Odometry/",
        "Drive/",
    ],
    "Vision": [
        "Vision/",
    ],
    "Shooter": [
        "Shooter/",
    ],
    "Intake": [
        "Intake/",
    ],
    "Hopper": [
        "Hopper/",
    ],
    "Superstructure": [
        "Superstructure/",
        "Robot/",
        "HubShift/",
    ],
}

# ---------------------------------------------------------------------------
# NT4 Connection Layer
# ---------------------------------------------------------------------------


class NT4Connection:
    """Manages a NetworkTables 4 client connection to the WPILib sim."""

    def __init__(self) -> None:
        self._inst = ntcore.NetworkTableInstance.create()
        self._inst.setServer(NT_SERVER_HOST, NT_SERVER_PORT)
        self._inst.startClient4("frc-telemetry-mcp")

        # Subscribe to everything under /AdvantageKit/
        self._multi_sub = ntcore.MultiSubscriber(
            self._inst, ["/AdvantageKit/"]
        )

        # Ring buffer: topic_name → deque of (timestamp_us, value)
        self._history: dict[str, deque[tuple[int, Any]]] = defaultdict(
            lambda: deque(maxlen=RING_BUFFER_MAX_SAMPLES)
        )

        # Cache of topic name → GenericSubscriber (avoid re-subscribing each poll)
        self._subscribers: dict[str, Any] = {}

        # Background thread to poll values into the ring buffer
        self._running = True
        self._poll_thread = threading.Thread(
            target=self._poll_loop, daemon=True
        )
        self._poll_thread.start()

    def _poll_loop(self) -> None:
        """Background thread that polls NT values into the ring buffer."""
        while self._running:
            try:
                if self.is_connected():
                    now = ntcore._now()
                    for topic in self._inst.getTopics():
                        name = topic.getName()
                        if not name.startswith("/AdvantageKit/"):
                            continue
                        # Cache subscribers to avoid creating new ones each cycle
                        if name not in self._subscribers:
                            self._subscribers[name] = topic.genericSubscribe()
                        val = self._subscribers[name].get()
                        if val.value() is not None:
                            serialized = _serialize_value(val.value(), name)
                            # Only record if value changed from last sample
                            buf = self._history[name]
                            if not buf or buf[-1][1] != serialized:
                                buf.append((now, serialized))
            except Exception:
                pass  # Graceful — don't crash the poll loop
            time.sleep(0.02)  # ~50Hz

    def is_connected(self) -> bool:
        return self._inst.isConnected()

    def get_connections(self) -> list[dict[str, Any]]:
        return [
            {
                "remote_id": c.remote_id,
                "remote_ip": c.remote_ip,
                "remote_port": c.remote_port,
            }
            for c in self._inst.getConnections()
        ]

    def get_all_topics(self) -> list[ntcore.TopicInfo]:
        return [
            t
            for t in self._inst.getTopics()
            if t.getName().startswith("/AdvantageKit/")
        ]

    def get_value(self, full_key: str) -> Any:
        """Get the current value of an NT key (full path)."""
        entry = self._inst.getTable("/").getEntry(full_key.lstrip("/"))
        val = entry.getValue()
        return _serialize_value(val.value(), full_key) if val.value() is not None else None

    def get_history(
        self, full_key: str, seconds: float = 5.0
    ) -> list[dict[str, Any]]:
        """Get recent history from the ring buffer."""
        buf = self._history.get(full_key)
        if not buf:
            return []
        now = ntcore._now()
        cutoff = now - int(seconds * 1_000_000)  # microseconds
        return [
            {"timestamp_us": ts, "value": v}
            for ts, v in buf
            if ts >= cutoff
        ]


    def close(self) -> None:
        self._running = False
        self._inst.stopClient()
        ntcore.NetworkTableInstance.destroy(self._inst)


import math
import struct as _struct


def _decode_pose3d_bytes(data: bytes) -> list[dict[str, float]] | None:
    """Decode a WPILib Pose3d[] struct blob.

    Each Pose3d is 56 bytes: 3 doubles (x,y,z) + 4 doubles (qw,qx,qy,qz), little-endian.
    Returns a list of pose dicts with x,y,z (m) and roll,pitch,yaw (deg), or None
    if the blob isn't a multiple of 56 bytes.
    """
    if not data or len(data) % 56 != 0:
        return None
    poses = []
    for i in range(0, len(data), 56):
        x, y, z, qw, qx, qy, qz = _struct.unpack_from("<7d", data, i)
        # Quaternion → RPY (ZYX intrinsic, matching WPILib Rotation3d.toRotationMatrix conversion)
        # roll (x), pitch (y), yaw (z)
        sinr = 2.0 * (qw * qx + qy * qz)
        cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr, cosr)
        sinp = 2.0 * (qw * qy - qz * qx)
        pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
        siny = 2.0 * (qw * qz + qx * qy)
        cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny, cosy)
        poses.append(
            {
                "x_m": round(x, 4),
                "y_m": round(y, 4),
                "z_m": round(z, 4),
                "roll_deg": round(math.degrees(roll), 3),
                "pitch_deg": round(math.degrees(pitch), 3),
                "yaw_deg": round(math.degrees(yaw), 3),
            }
        )
    return poses


def _serialize_value(value: Any, key_hint: str = "") -> Any:
    """Convert NT values to JSON-safe Python types."""
    if isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, (list, tuple)):
        return [_serialize_value(v) for v in value]
    if isinstance(value, bytes):
        # Auto-decode Pose3d/Pose3d[] struct blobs (key name contains Pose and size is 56*N)
        if ("Pose" in key_hint or "Tag" in key_hint) and len(value) % 56 == 0 and len(value) > 0:
            poses = _decode_pose3d_bytes(value)
            if poses is not None:
                return {"poses": poses, "count": len(poses)}
        return f"<bytes:{len(value)}>"
    # Fallback
    return str(value)


# ---------------------------------------------------------------------------
# Key Resolution
# ---------------------------------------------------------------------------


def _resolve_key(nt: NT4Connection, short_key: str) -> str | None:
    """
    Resolve a short-form key to a full NT path.

    Accepts:
      - Full path: "/AdvantageKit/RealOutputs/Odometry/Robot"
      - Short form: "Odometry/Robot"

    Tries /AdvantageKit/RealOutputs/<key> first, then /AdvantageKit/<key>,
    then searches all topics for a suffix match.
    """
    if short_key.startswith("/AdvantageKit/"):
        return short_key

    # Try outputs first, then raw
    for prefix in [AK_OUTPUTS_PREFIX, AK_INPUTS_PREFIX]:
        candidate = prefix + short_key
        # Check if topic exists
        topics = nt._inst.getTopics()
        for t in topics:
            if t.getName() == candidate:
                return candidate

    # Fallback: suffix match
    for t in nt._inst.getTopics():
        name = t.getName()
        if name.endswith("/" + short_key) or name.endswith(short_key):
            return name

    return None


def _resolve_key_or_error(nt: NT4Connection, key: str) -> str:
    """Resolve a key or raise a descriptive error."""
    resolved = _resolve_key(nt, key)
    if resolved is None:
        raise ValueError(
            f"Key '{key}' not found in NetworkTables. "
            f"Use list_topics() to see available keys."
        )
    return resolved


# ---------------------------------------------------------------------------
# MCP Server
# ---------------------------------------------------------------------------

mcp = FastMCP(
    "FRC Telemetry",
    instructions=(
        "This server provides real-time telemetry from an FRC robot simulation "
        "running in WPILib. Use these tools to read sensor data, motor states, "
        "odometry, vision, and subsystem states to diagnose issues and iterate "
        "on robot code. All data comes from AdvantageKit via NetworkTables 4."
    ),
)

# Global NT connection — initialized lazily
_nt: NT4Connection | None = None


def _get_nt() -> NT4Connection:
    global _nt
    if _nt is None:
        _nt = NT4Connection()
        # Give it a moment to discover topics
        time.sleep(0.5)
    return _nt


# ---------------------------------------------------------------------------
# Tool: get_connection_status
# ---------------------------------------------------------------------------


@mcp.tool()
def get_connection_status() -> str:
    """
    Check if the MCP server is connected to the WPILib sim's NetworkTables server.

    Returns connection status, number of available topics, and connection info.
    Use this first to verify the sim is running before querying telemetry.
    """
    nt = _get_nt()
    connected = nt.is_connected()
    topics = nt.get_all_topics()
    connections = nt.get_connections()

    result = {
        "connected": connected,
        "server": f"{NT_SERVER_HOST}:{NT_SERVER_PORT}",
        "topic_count": len(topics),
        "connections": connections,
    }
    if not connected:
        result["hint"] = (
            "The sim does not appear to be running. "
            "Start it with: ./gradlew simulateJava"
        )
    return json.dumps(result, indent=2)


# ---------------------------------------------------------------------------
# Tool: list_topics
# ---------------------------------------------------------------------------


@mcp.tool()
def list_topics(prefix: str = "") -> str:
    """
    List all available NetworkTables topics from the robot sim.

    Args:
        prefix: Optional filter — only return topics containing this string.
                Examples: "Drive", "Shooter", "Odometry", "Vision"

    Returns a list of topic names and their NT types.
    Use this to discover what telemetry data is available before reading values.
    """
    nt = _get_nt()
    if not nt.is_connected():
        return json.dumps({"error": "Not connected to sim. Is it running?"})

    topics = nt.get_all_topics()
    results = []
    for t in topics:
        name = t.getName()
        # Strip the AK prefix for readability
        short = name
        if name.startswith(AK_OUTPUTS_PREFIX):
            short = name[len(AK_OUTPUTS_PREFIX) :]
        elif name.startswith(AK_INPUTS_PREFIX):
            short = name[len(AK_INPUTS_PREFIX) :]

        if prefix and prefix.lower() not in short.lower():
            continue

        results.append(
            {
                "key": short,
                "full_path": name,
                "type": t.getTypeString(),
            }
        )

    results.sort(key=lambda x: x["key"])
    return json.dumps(results, indent=2)


# ---------------------------------------------------------------------------
# Tool: get_value
# ---------------------------------------------------------------------------


@mcp.tool()
def get_value(key: str) -> str:
    """
    Get the current value of a single NetworkTables key.

    Args:
        key: The topic key. Accepts short form (e.g., "Odometry/Robot") or
             full path (e.g., "/AdvantageKit/RealOutputs/Odometry/Robot").

    Returns the current value, or an error if the key is not found.
    """
    nt = _get_nt()
    if not nt.is_connected():
        return json.dumps({"error": "Not connected to sim. Is it running?"})

    try:
        full_key = _resolve_key_or_error(nt, key)
    except ValueError as e:
        return json.dumps({"error": str(e)})

    value = nt.get_value(full_key)
    return json.dumps({"key": key, "full_path": full_key, "value": value}, indent=2)


# ---------------------------------------------------------------------------
# Tool: get_values
# ---------------------------------------------------------------------------


@mcp.tool()
def get_values(keys: list[str]) -> str:
    """
    Get current values for multiple NetworkTables keys at once.

    Args:
        keys: List of topic keys (short or full form).
              Example: ["Odometry/Robot", "SwerveChassisSpeeds/Measured"]

    Returns a dict of key → value for each requested key.
    """
    nt = _get_nt()
    if not nt.is_connected():
        return json.dumps({"error": "Not connected to sim. Is it running?"})

    results = {}
    for key in keys:
        resolved = _resolve_key(nt, key)
        if resolved:
            results[key] = nt.get_value(resolved)
        else:
            results[key] = f"NOT_FOUND"

    return json.dumps(results, indent=2)


# ---------------------------------------------------------------------------
# Tool: get_subsystem_snapshot
# ---------------------------------------------------------------------------


@mcp.tool()
def get_subsystem_snapshot(subsystem: str) -> str:
    """
    Get all telemetry values for a robot subsystem.

    Args:
        subsystem: Name of subsystem. Supported values:
                   "Drive" — swerve module states, chassis speeds, odometry, gyro
                   "Vision" — camera observations, pose estimates, tag data
                   "Shooter" — flywheel RPM, hood angle, kicker, state machine
                   "Intake" — roller/arm positions, velocities, currents
                   "Hopper" — motor states, currents
                   "Superstructure" — robot state, zone, hub shift, alliance

    If the subsystem name is not recognized, falls back to prefix matching
    against all available topics.
    """
    nt = _get_nt()
    if not nt.is_connected():
        return json.dumps({"error": "Not connected to sim. Is it running?"})

    prefixes = SUBSYSTEM_PREFIXES.get(subsystem)
    topics = nt.get_all_topics()
    results = {}

    for t in topics:
        name = t.getName()
        # Get the short name
        short = name
        if name.startswith(AK_OUTPUTS_PREFIX):
            short = name[len(AK_OUTPUTS_PREFIX) :]
        elif name.startswith(AK_INPUTS_PREFIX):
            short = name[len(AK_INPUTS_PREFIX) :]

        match = False
        if prefixes:
            match = any(short.startswith(p) for p in prefixes)
        else:
            # Fallback: prefix match on subsystem name
            match = short.lower().startswith(subsystem.lower())

        if match:
            value = nt.get_value(name)
            results[short] = value

    if not results:
        available = list(SUBSYSTEM_PREFIXES.keys())
        return json.dumps(
            {
                "error": f"No data found for subsystem '{subsystem}'.",
                "available_subsystems": available,
                "hint": "Use list_topics() to discover available data.",
            },
            indent=2,
        )

    return json.dumps(
        {"subsystem": subsystem, "values": results},
        indent=2,
    )


# ---------------------------------------------------------------------------
# Tool: get_full_snapshot
# ---------------------------------------------------------------------------


@mcp.tool()
def get_full_snapshot() -> str:
    """
    Get ALL current AdvantageKit telemetry values from the robot sim.

    Returns every topic and its current value, grouped by prefix.
    Warning: this can be large. Prefer get_subsystem_snapshot() for focused queries.
    """
    nt = _get_nt()
    if not nt.is_connected():
        return json.dumps({"error": "Not connected to sim. Is it running?"})

    topics = nt.get_all_topics()
    results = {}
    for t in topics:
        name = t.getName()
        short = name
        if name.startswith(AK_OUTPUTS_PREFIX):
            short = name[len(AK_OUTPUTS_PREFIX) :]
        elif name.startswith(AK_INPUTS_PREFIX):
            short = name[len(AK_INPUTS_PREFIX) :]

        value = nt.get_value(name)
        results[short] = value

    return json.dumps(
        {"topic_count": len(results), "values": results},
        indent=2,
    )


# ---------------------------------------------------------------------------
# Tool: get_value_history
# ---------------------------------------------------------------------------


@mcp.tool()
def get_value_history(key: str, seconds: float = 5.0) -> str:
    """
    Get recent time-series history of a NetworkTables value.

    Args:
        key: The topic key (short or full form).
        seconds: How many seconds of history to return (default 5, max 10).

    Returns timestamped samples from the ring buffer. Critical for diagnosing
    oscillations, PID tuning issues, convergence problems, and timing delays.
    If few/no samples are returned, the key may not be updating or the sim
    may have just started.
    """
    nt = _get_nt()
    if not nt.is_connected():
        return json.dumps({"error": "Not connected to sim. Is it running?"})

    seconds = min(seconds, RING_BUFFER_SECONDS)

    try:
        full_key = _resolve_key_or_error(nt, key)
    except ValueError as e:
        return json.dumps({"error": str(e)})

    history = nt.get_history(full_key, seconds)

    return json.dumps(
        {
            "key": key,
            "full_path": full_key,
            "seconds_requested": seconds,
            "sample_count": len(history),
            "samples": history,
        },
        indent=2,
    )


# ---------------------------------------------------------------------------
# Tool: get_drive_diagnostics
# ---------------------------------------------------------------------------


@mcp.tool()
def get_drive_diagnostics() -> str:
    """
    Get a comprehensive drive subsystem diagnostic snapshot.

    Returns current pose, measured vs setpoint chassis speeds, all swerve module
    states, angular velocity, and gyro data. Pre-formatted for debugging drive
    behavior such as heading overshoot, drift, module alignment, or path following.
    """
    nt = _get_nt()
    if not nt.is_connected():
        return json.dumps({"error": "Not connected to sim. Is it running?"})

    keys_to_read = [
        "Odometry/Robot",
        "SwerveStates/Measured",
        "SwerveStates/Setpoints",
        "SwerveStates/SetpointsOptimized",
        "SwerveChassisSpeeds/Measured",
        "SwerveChassisSpeeds/Setpoints",
        "Drive/SysIdState",
    ]

    # Also grab gyro inputs
    gyro_keys = [
        "Drive/Gyro/Connected",
        "Drive/Gyro/YawPosition",
        "Drive/Gyro/YawVelocityRadPerSec",
    ]

    results = {}
    for key in keys_to_read + gyro_keys:
        resolved = _resolve_key(nt, key)
        if resolved:
            results[key] = nt.get_value(resolved)

    return json.dumps(
        {"diagnostic": "Drive", "values": results},
        indent=2,
    )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main():
    mcp.run(transport="stdio")


if __name__ == "__main__":
    main()
