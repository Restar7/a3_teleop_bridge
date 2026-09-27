"""Online teleop pipeline: latest-only queues, worker threads, state machine.

Plan sections 29-32 and 73.  This module owns the *architecture* of the live
path; the actual SMPL -> A3 solver is pluggable through :class:`RetargetBackend`
so the threading/queueing behaviour can be tested without a GPU or a headset:

    PICO receiver thread ──┐ (queue maxsize=1, latest only)
                           ▼
                    solver thread (RetargetBackend.step)
                           │ (queue maxsize=1, latest only)
                           ▼
                 predictor thread -> ReferencePublisher

Rules enforced here:

* **no unbounded queues** -- every hand-off keeps exactly one item, so a slow
  solver drops stale frames instead of building a backlog;
* **timestamps everywhere** -- every stage records source/receive/solve/publish
  timestamps and the resulting latencies (plan section 32);
* **explicit state machine** -- DISCONNECTED -> CALIBRATION -> READY ->
  TRACKING -> HOLD -> SAFE_STOP, with the transitions driven by data freshness
  rather than by the mere existence of a connection (plan section 73).
"""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Protocol

import numpy as np

from ..clocks import LatencyStats, now_ns
from ..contract import load_contract
from ..types import A3CanonicalState, A3ReferenceWindow, BridgeState, HumanSmplFrame

__all__ = [
    "RetargetBackend",
    "LatestSlot",
    "StageStats",
    "OnlineTeleopPipeline",
]


class RetargetBackend(Protocol):
    """Anything that turns a human SMPL frame into an A3 canonical state."""

    def reset(self, initial_frame: HumanSmplFrame | None = None) -> None:
        ...

    def step(self, frame: HumanSmplFrame, timestamp_ns: int) -> A3CanonicalState:
        ...


@dataclass
class LatestSlot:
    """A one-element mailbox: writers never block, readers always get the newest."""

    dropped: int = 0
    _item: object | None = None
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)
    _event: threading.Event = field(default_factory=threading.Event, repr=False)

    def put(self, item) -> None:
        with self._lock:
            if self._item is not None:
                self.dropped += 1
            self._item = item
            self._event.set()

    def get(self, timeout: float | None = None):
        """Block until an item is available; returns None on timeout."""
        if not self._event.wait(timeout):
            return None
        with self._lock:
            item = self._item
            self._item = None
            self._event.clear()
            return item

    def peek(self):
        with self._lock:
            return self._item

    def clear(self) -> None:
        with self._lock:
            self._item = None
            self._event.clear()


@dataclass
class StageStats:
    """Per-stage latency accounting shared by the pipeline and benchmarks."""

    frames_in: int = 0
    frames_solved: int = 0
    frames_published: int = 0
    rejected: int = 0
    solver_latency_ms: LatencyStats = field(default_factory=lambda: LatencyStats("umr"))
    predictor_latency_ms: LatencyStats = field(default_factory=lambda: LatencyStats("predictor"))
    end_to_end_ms: LatencyStats = field(default_factory=lambda: LatencyStats("end_to_end"))

    def as_dict(self) -> dict:
        return {
            "frames_in": self.frames_in,
            "frames_solved": self.frames_solved,
            "frames_published": self.frames_published,
            "rejected": self.rejected,
            "solver_latency_ms": self.solver_latency_ms.summary(),
            "predictor_latency_ms": self.predictor_latency_ms.summary(),
            "end_to_end_ms": self.end_to_end_ms.summary(),
        }


class OnlineTeleopPipeline:
    """Owns the threads, the latest-only hand-offs and the state machine."""

    def __init__(
        self,
        backend: RetargetBackend,
        predictor,
        publisher=None,
        *,
        hold_after_ms: float = 50.0,
        invalid_after_ms: float = 250.0,
        stats: StageStats | None = None,
    ) -> None:
        self.backend = backend
        self.predictor = predictor
        self.publisher = publisher
        self.hold_after_ms = float(hold_after_ms)
        self.invalid_after_ms = float(invalid_after_ms)
        self.stats = stats or StageStats()
        self.contract = load_contract()

        self.human_slot = LatestSlot()
        self.state_slot = LatestSlot()
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self.state = BridgeState.DISCONNECTED
        self.state_history: list[tuple[str, int]] = [(self.state.value, now_ns())]
        self.last_frame_timestamp_ns = 0
        self.calibration = None

    # ------------------------------------------------------------------
    def set_state(self, state: BridgeState) -> None:
        if state is not self.state:
            self.state = state
            self.state_history.append((state.value, now_ns()))

    def _maybe_transition(self) -> None:
        """Data-driven transitions (plan section 73)."""
        if self.last_frame_timestamp_ns == 0:
            self.set_state(BridgeState.DISCONNECTED)
            return
        age_ms = (now_ns() - self.last_frame_timestamp_ns) / 1e6
        if age_ms > self.invalid_after_ms:
            self.set_state(BridgeState.SAFE_STOP)
        elif age_ms > self.hold_after_ms:
            self.set_state(BridgeState.HOLD)
        elif self.calibration is None:
            self.set_state(BridgeState.CALIBRATION)
        else:
            self.set_state(BridgeState.TRACKING)

    # ------------------------------------------------------------------
    # producer
    def submit(self, frame: HumanSmplFrame) -> None:
        """Called by the receiver thread; never blocks."""
        self.stats.frames_in += 1
        self.last_frame_timestamp_ns = frame.timestamp_ns or now_ns()
        self.human_slot.put(frame)

    # ------------------------------------------------------------------
    # workers
    def _solver_loop(self) -> None:
        while not self._stop.is_set():
            frame = self.human_slot.get(timeout=0.05)
            if frame is None:
                continue
            started = time.perf_counter()
            try:
                state = self.backend.step(frame, now_ns())
            except Exception as exc:  # solver failure must not kill the loop
                self.stats.rejected += 1
                state = A3CanonicalState.invalid(
                    seq=self.stats.frames_solved,
                    timestamp_ns=now_ns(),
                    n_joints=self.contract.n_policy_joints,
                    reason=f"solver raised: {exc}",
                )
            self.stats.solver_latency_ms.add((time.perf_counter() - started) * 1e3)
            self.stats.frames_solved += 1
            self.state_slot.put(state)

    def _predictor_loop(self) -> None:
        while not self._stop.is_set():
            state = self.state_slot.get(timeout=0.05)
            if state is None:
                continue
            started = time.perf_counter()
            if state.valid:
                self.predictor.push(state)
            self._maybe_transition()
            age_ms = (
                (now_ns() - self.last_frame_timestamp_ns) / 1e6
                if self.last_frame_timestamp_ns
                else float("inf")
            )
            window = self.predictor.window(timestamp_ns=now_ns(), source_age_ms=age_ms)
            window.state = self.state
            self.stats.predictor_latency_ms.add((time.perf_counter() - started) * 1e3)
            if state.timestamp_ns:
                self.stats.end_to_end_ms.add((now_ns() - state.timestamp_ns) / 1e6)
            if self.publisher is not None:
                try:
                    self.publisher.send(window)
                    self.stats.frames_published += 1
                except Exception:
                    self.stats.rejected += 1

    # ------------------------------------------------------------------
    def start(self, initial_frame: HumanSmplFrame | None = None) -> None:
        self.backend.reset(initial_frame)
        self._stop.clear()
        self._threads = [
            threading.Thread(target=self._solver_loop, name="a3-solver", daemon=True),
            threading.Thread(target=self._predictor_loop, name="a3-predictor", daemon=True),
        ]
        for thread in self._threads:
            thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=timeout)
        self._threads = []

    def run_for(
        self,
        frames: Callable[[], HumanSmplFrame | None],
        duration_s: float,
        poll_interval_s: float = 0.001,
    ) -> StageStats:
        """Drive the pipeline from a frame producer for ``duration_s`` seconds."""
        deadline = time.perf_counter() + duration_s
        while time.perf_counter() < deadline and not self._stop.is_set():
            frame = frames()
            if frame is None:
                time.sleep(poll_interval_s)
                continue
            self.submit(frame)
        return self.stats

    def __enter__(self) -> "OnlineTeleopPipeline":
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.stop()
