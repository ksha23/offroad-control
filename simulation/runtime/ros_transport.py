"""ROS 2 (rclpy) transport backend mirroring the ZMQPublisher/ZMQSubscriber
interface in ``hil_messages.py``.

This is the ROS-native transport for the decoupled HIL stack. It carries the
SAME framed message bytes (``msg.to_bytes()`` / ``parse_message``) as the ZMQ
backend, so the higher layers are transport-agnostic -- the only difference is
DDS instead of ZeroMQ pub/sub.

Design choices matching the ZMQ semantics (see the ZMQ classes for rationale):
  * One ROS topic per *port* (``/hil/port_<n>``), preserving the ZMQ scheme
    where a single socket multiplexes several message types (e.g. port 5555
    carries both ``vehicle_state`` and ``sim_status``); the per-message framing
    in ``to_bytes()`` still distinguishes them via ``parse_message``.
  * QoS ``KEEP_LAST`` depth 1 + ``BEST_EFFORT`` == ZeroMQ ``CONFLATE=1`` +
    ``RCVHWM=2`` (latest-only, drop stale) -- the property the control loop
    relies on so the consumer never acts on a queued/stale command.
  * Parallel sweep workers are isolated by ``ROS_DOMAIN_ID`` (set per worker),
    the ROS analogue of the ZMQ per-worker port block.

The payload is wrapped in ``std_msgs/UInt8MultiArray`` (raw bytes), so no custom
.msg / rosidl build step is required -- the framing + msgpack in
``hil_messages`` remains the single source of truth for the schema.
"""
from __future__ import annotations

import threading
from typing import List, Optional

# rclpy + std_msgs are only imported when the ROS backend is actually used, so
# the ZMQ path never requires ROS 2 to be sourced.
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import UInt8MultiArray

from hil_messages import parse_message

# Latest-only QoS: the DDS analogue of ZeroMQ CONFLATE=1 (depth-1 keep-last).
_LATEST_ONLY = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
)

# One rclpy context + node + executor per PROCESS, shared by every
# publisher/subscriber the process creates (mirrors zmq.Context.instance()).
# We deliberately DO NOT run a background spin thread: rclpy's executor teardown
# races a daemon spin thread and aborts (core dump) at process exit, which would
# fail sweep runs. Instead subscribers spin the executor inside recv() -- this
# also matches the ZMQ polling model (consumers poll each loop iteration).
_LOCK = threading.Lock()
_NODE: Optional[Node] = None
_EXECUTOR: Optional[SingleThreadedExecutor] = None
_REFCOUNT = 0


def _port_topic(endpoint: str) -> str:
    """Map a ZMQ-style endpoint (``tcp://host:PORT``) to a ROS topic name."""
    port = endpoint.rsplit(":", 1)[-1].strip("/")
    return f"/hil/port_{port}"


def _ensure_node() -> Node:
    """Lazily start the shared process node + executor (no spin thread)."""
    global _NODE, _EXECUTOR, _REFCOUNT
    with _LOCK:
        if _NODE is None:
            if not rclpy.ok():
                rclpy.init(args=None)
            _NODE = Node("hil_transport")
            _EXECUTOR = SingleThreadedExecutor()
            _EXECUTOR.add_node(_NODE)
        _REFCOUNT += 1
        return _NODE


def _spin(timeout_sec: float) -> None:
    """Process pending callbacks on the shared executor (called from recv)."""
    if _EXECUTOR is not None:
        _EXECUTOR.spin_once(timeout_sec=timeout_sec)


def _release_node() -> None:
    global _NODE, _EXECUTOR, _REFCOUNT
    with _LOCK:
        _REFCOUNT -= 1
        if _REFCOUNT <= 0 and _NODE is not None:
            try:
                _NODE.destroy_node()
            except Exception:
                pass
            _NODE = None
            _EXECUTOR = None
            _REFCOUNT = 0
            if rclpy.ok():
                rclpy.shutdown()


class ROSPublisher:
    """Publish framed HIL messages on a ROS 2 topic (ZMQPublisher analogue)."""

    def __init__(self, endpoint: str = "tcp://*:5555", topic: Optional[str] = None):
        self._node = _ensure_node()
        # Prefer an explicit semantic topic (e.g. /scm_hil/vehicle_state); fall
        # back to a port-derived name. Parallel runs are separated by
        # ROS_DOMAIN_ID, so fixed semantic topics don't collide across runs.
        self.topic = topic or _port_topic(endpoint)
        self.endpoint = endpoint
        self._pub = self._node.create_publisher(UInt8MultiArray, self.topic, _LATEST_ONLY)
        self._closed = False

    def send(self, msg) -> None:
        """Send a dataclass message (must have ``.to_bytes()``)."""
        m = UInt8MultiArray()
        m.data = list(msg.to_bytes())
        self._pub.publish(m)

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            self._node.destroy_publisher(self._pub)
        except Exception:
            pass
        _release_node()


class ROSSubscriber:
    """Subscribe to framed HIL messages on a ROS 2 topic (ZMQSubscriber analogue).

    Keeps only the latest received frame (conflating), matching the ZMQ
    ``CONFLATE=1`` subscriber. ``recv`` returns the newest un-consumed frame.
    """

    def __init__(self, endpoint: str = "tcp://localhost:5555",
                 topics: Optional[List[str]] = None, topic: Optional[str] = None):
        self._node = _ensure_node()
        self.topic = topic or _port_topic(endpoint)
        self.endpoint = endpoint
        self._latest: Optional[bytes] = None
        self._lock = threading.Lock()
        self._closed = False
        self._sub = self._node.create_subscription(
            UInt8MultiArray, self.topic, self._on_msg, _LATEST_ONLY)

    def _on_msg(self, m: UInt8MultiArray) -> None:
        with self._lock:
            self._latest = bytes(bytearray(m.data))  # conflate: overwrite

    def recv(self, timeout_ms: int = 0):
        """Return (topic, msg) for the latest frame, or None.

        Spins the shared executor to process any pending DDS callbacks (which
        overwrite ``_latest``, conflating to newest), then hands off the latest.
        timeout_ms==0 is a non-blocking poll; >0 waits up to that long for a
        frame; matches ZMQSubscriber.recv semantics."""
        # QoS KEEP_LAST depth 1 means the middleware holds only the newest
        # sample, so one spin_once processes it. Non-blocking poll uses 0.0;
        # a positive timeout blocks up to that long inside the executor.
        _spin(0.0 if timeout_ms <= 0 else timeout_ms / 1000.0)
        with self._lock:
            raw, self._latest = self._latest, None
        if raw is None:
            return None
        try:
            return parse_message(raw)
        except Exception:
            return None

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            self._node.destroy_subscription(self._sub)
        except Exception:
            pass
        _release_node()
