"""Timestamp, rate and latency bookkeeping (plan sections 32 and 46).

Everything in the live path must carry timestamps; ``time.sleep`` is never the
sync mechanism.
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field

import numpy as np

__all__ = [
    "now_ns",
    "ns_to_ms",
    "ms_to_ns",
    "RateMeter",
    "LatencyStats",
    "SeqTracker",
    "StaleWatchdog",
]


def now_ns() -> int:
    """Monotonic-ish wall clock in nanoseconds (time.time_ns base)."""
    return time.time_ns()


def ns_to_ms(delta_ns: float) -> float:
    return float(delta_ns) / 1e6


def ms_to_ns(delta_ms: float) -> int:
    return int(round(float(delta_ms) * 1e6))


@dataclass
class RateMeter:
    """Rolling rate / jitter estimator over the last ``window`` timestamps."""

    window: int = 200
    _stamps: deque = field(default_factory=deque, repr=False)

    def tick(self, timestamp_ns: int | None = None) -> None:
        self._stamps.append(now_ns() if timestamp_ns is None else int(timestamp_ns))
        while len(self._stamps) > self.window:
            self._stamps.popleft()

    @property
    def count(self) -> int:
        return len(self._stamps)

    @property
    def hz(self) -> float:
        if len(self._stamps) < 2:
            return 0.0
        span_s = (self._stamps[-1] - self._stamps[0]) / 1e9
        if span_s <= 0:
            return 0.0
        return (len(self._stamps) - 1) / span_s

    @property
    def period_ms_mean(self) -> float:
        if len(self._stamps) < 2:
            return 0.0
        diffs = np.diff(np.asarray(self._stamps, dtype=np.float64))
        return float(diffs.mean()) / 1e6

    def reset(self) -> None:
        self._stamps.clear()


@dataclass
class LatencyStats:
    """Percentile summary used by ``tools/benchmark_latency.py``."""

    name: str = "latency"
    unit: str = "ms"
    _samples: list[float] = field(default_factory=list, repr=False)

    def add(self, value_ms: float) -> None:
        if np.isfinite(value_ms):
            self._samples.append(float(value_ms))

    def extend(self, values) -> None:
        for value in values:
            self.add(float(value))

    def __len__(self) -> int:
        return len(self._samples)

    def summary(self) -> dict[str, float]:
        if not self._samples:
            return {
                "count": 0,
                "mean": float("nan"),
                "p50": float("nan"),
                "p90": float("nan"),
                "p95": float("nan"),
                "p99": float("nan"),
                "max": float("nan"),
            }
        arr = np.asarray(self._samples, dtype=np.float64)
        return {
            "count": int(arr.size),
            "mean": float(arr.mean()),
            "p50": float(np.percentile(arr, 50)),
            "p90": float(np.percentile(arr, 90)),
            "p95": float(np.percentile(arr, 95)),
            "p99": float(np.percentile(arr, 99)),
            "max": float(arr.max()),
        }

    def to_dict(self) -> dict:
        return {"name": self.name, "unit": self.unit, **self.summary()}


@dataclass
class SeqTracker:
    """Detect dropped, duplicated and out-of-order sequence numbers."""

    expected: int | None = None
    received: int = 0
    dropped: int = 0
    duplicates: int = 0
    reordered: int = 0
    last_seq: int | None = None

    def update(self, seq: int) -> str:
        """Feed one sequence number; returns one of new/duplicate/reordered/dropped."""
        self.received += 1
        outcome = "new"
        if self.last_seq is not None:
            if seq == self.last_seq:
                self.duplicates += 1
                outcome = "duplicate"
            elif seq < self.last_seq:
                self.reordered += 1
                outcome = "reordered"
            elif seq > self.last_seq + 1:
                self.dropped += seq - self.last_seq - 1
        if self.expected is not None and seq < self.expected:
            if outcome == "new":
                outcome = "reordered"
        self.last_seq = max(seq, self.last_seq) if self.last_seq is not None else seq
        return outcome

    def stats(self) -> dict[str, int]:
        return {
            "received": self.received,
            "dropped": self.dropped,
            "duplicates": self.duplicates,
            "reordered": self.reordered,
        }


@dataclass
class StaleWatchdog:
    """Age-based watchdog with HOLD / INVALID thresholds (plan section 36)."""

    hold_after_ms: float = 50.0
    invalid_after_ms: float = 250.0

    def classify(self, age_ms: float) -> str:
        if not np.isfinite(age_ms):
            return "INVALID"
        if age_ms >= self.invalid_after_ms:
            return "INVALID"
        if age_ms >= self.hold_after_ms:
            return "HOLD"
        return "OK"
