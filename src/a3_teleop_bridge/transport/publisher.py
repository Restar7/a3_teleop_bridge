"""ZMQ reference publisher / subscriber with latest-only semantics (plan §38)."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path

import yaml
import zmq

from ..clocks import SeqTracker, now_ns
from ..contract import A3Contract, load_contract
from ..types import A3ReferenceWindow
from .protocol import ProtocolError, decode_any, decode_packet, encode_packet

BRIDGE_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_NETWORK_CONFIG = BRIDGE_ROOT / "configs" / "network.yaml"

__all__ = [
    "NetworkConfig",
    "ReferencePublisher",
    "ReferenceSubscriber",
    "ReceivedWindow",
]


@dataclass
class NetworkConfig:
    bind_host: str = "0.0.0.0"
    connect_host: str = "127.0.0.1"
    reference_port: int = 5560
    pico_port: int = 5556
    high_water_mark: int = 1
    conflate: bool = True
    recv_timeout_ms: int = 50
    raw: dict = field(default_factory=dict)

    @classmethod
    def from_yaml(cls, path: Path | str | None = None) -> "NetworkConfig":
        path = Path(path).expanduser() if path else DEFAULT_NETWORK_CONFIG
        if not path.is_file():
            raise FileNotFoundError(f"network config not found: {path}")
        with open(path, "r", encoding="utf-8") as handle:
            doc = yaml.safe_load(handle) or {}
        reference = doc.get("reference", {}) or {}
        pico = doc.get("pico", {}) or {}
        zmq_doc = doc.get("zmq", {}) or {}
        return cls(
            bind_host=str(reference.get("bind_host", "0.0.0.0")),
            connect_host=str(reference.get("connect_host", "127.0.0.1")),
            reference_port=int(reference.get("port", 5560)),
            pico_port=int(pico.get("port", 5556)),
            high_water_mark=int(zmq_doc.get("high_water_mark", 1)),
            conflate=bool(zmq_doc.get("conflate", True)),
            recv_timeout_ms=int(zmq_doc.get("recv_timeout_ms", 50)),
            raw=doc,
        )

    @property
    def reference_bind(self) -> str:
        return f"tcp://{self.bind_host}:{self.reference_port}"

    @property
    def reference_connect(self) -> str:
        return f"tcp://{self.connect_host}:{self.reference_port}"

    @property
    def pico_bind(self) -> str:
        return f"tcp://{self.bind_host}:{self.pico_port}"


class ReferencePublisher:
    """PUB socket that never queues more than one reference window."""

    def __init__(
        self,
        config: NetworkConfig | None = None,
        contract: A3Contract | None = None,
        context: zmq.Context | None = None,
        bind: str | None = None,
    ) -> None:
        self.config = config or NetworkConfig.from_yaml()
        self.contract = contract or load_contract()
        self.context = context or zmq.Context.instance()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, self.config.high_water_mark)
        if self.config.conflate:
            self.socket.setsockopt(zmq.CONFLATE, 1)
        self.bind = bind or self.config.reference_bind
        self.socket.bind(self.bind)
        self.sent = 0
        self.last_seq = -1

    def send(self, window: A3ReferenceWindow) -> int:
        """Publish one window as a single (conflatable) ZMQ frame."""
        packet = encode_packet(window, self.contract)
        self.socket.send(packet)
        self.sent += 1
        self.last_seq = int(window.seq)
        return len(packet)

    def close(self) -> None:
        self.socket.close(linger=0)

    def __enter__(self) -> "ReferencePublisher":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


@dataclass
class ReceivedWindow:
    window: A3ReferenceWindow
    receive_timestamp_ns: int
    network_latency_ms: float
    frames: int
    size_bytes: int
    rejected: bool = False
    reject_reason: str = ""


class ReferenceSubscriber:
    """SUB socket that only ever exposes the newest window (latest-only)."""

    def __init__(
        self,
        config: NetworkConfig | None = None,
        contract: A3Contract | None = None,
        context: zmq.Context | None = None,
        connect: str | None = None,
        subscribe: bool = True,
    ) -> None:
        self.config = config or NetworkConfig.from_yaml()
        self.contract = contract or load_contract()
        self.context = context or zmq.Context.instance()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.RCVHWM, self.config.high_water_mark)
        if self.config.conflate:
            self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.setsockopt(zmq.RCVTIMEO, self.config.recv_timeout_ms)
        self.connect = connect or self.config.reference_connect
        if subscribe:
            self.socket.setsockopt(zmq.SUBSCRIBE, b"")
            self.socket.connect(self.connect)
        self.seq_tracker = SeqTracker()
        self.received = 0
        self.rejected = 0
        self.last: ReceivedWindow | None = None

    # ------------------------------------------------------------------
    def poll(self, timeout_ms: int | None = None) -> ReceivedWindow | None:
        """Return the newest available window, draining older ones."""
        timeout = self.config.recv_timeout_ms if timeout_ms is None else timeout_ms
        packet = None
        try:
            packet = self.socket.recv(zmq.NOBLOCK)
            # latest-only: keep draining until the socket is empty
            while True:
                try:
                    packet = self.socket.recv(zmq.NOBLOCK)
                except zmq.Again:
                    break
        except zmq.Again:
            if timeout and timeout > 0:
                if self.socket.poll(timeout) == 0:
                    return None
                try:
                    packet = self.socket.recv(zmq.NOBLOCK)
                except zmq.Again:
                    return None
            if packet is None:
                return None

        size = len(packet)
        received_ns = now_ns()
        try:
            window = decode_packet(packet, self.contract)
        except ProtocolError as exc:
            self.rejected += 1
            self.last = ReceivedWindow(
                window=None,  # type: ignore[arg-type]
                receive_timestamp_ns=received_ns,
                network_latency_ms=float("nan"),
                frames=1,
                size_bytes=size,
                rejected=True,
                reject_reason=str(exc),
            )
            return self.last

        self.received += 1
        self.seq_tracker.update(window.seq)
        latency_ms = (received_ns - window.timestamp_ns) / 1e6 if window.timestamp_ns else float("nan")
        self.last = ReceivedWindow(
            window=window,
            receive_timestamp_ns=received_ns,
            network_latency_ms=latency_ms,
            frames=1,
            size_bytes=size,
        )
        return self.last

    def stats(self) -> dict:
        return {
            "received": self.received,
            "rejected": self.rejected,
            **self.seq_tracker.stats(),
        }

    def close(self) -> None:
        self.socket.close(linger=0)

    def __enter__(self) -> "ReferenceSubscriber":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def wait_for_connection(subscriber: ReferenceSubscriber, timeout_s: float = 5.0) -> bool:
    """Block until the first window arrives (or timeout)."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        received = subscriber.poll(timeout_ms=100)
        if received is not None and not received.rejected:
            return True
    return False
