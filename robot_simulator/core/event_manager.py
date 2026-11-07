"""Simple publish/subscribe events and a log buffer in memory."""
from __future__ import annotations

import logging
import threading
import time
from collections import defaultdict, deque
from typing import Any, Callable

log = logging.getLogger(__name__)


class EventManager:
    """Synchronous events. The thread that emits an event (the main loop) calls the handlers."""

    def __init__(self):
        self._subs: dict[str, list[Callable[..., Any]]] = defaultdict(list)

    def subscribe(self, event: str, callback: Callable[..., Any]) -> None:
        self._subs[event].append(callback)

    def unsubscribe(self, event: str, callback: Callable[..., Any]) -> None:
        if callback in self._subs.get(event, []):
            self._subs[event].remove(callback)

    def emit(self, event: str, *args, **kwargs) -> None:
        for cb in list(self._subs.get(event, ())):
            try:
                cb(*args, **kwargs)
            except Exception:
                log.exception("Event handler for '%s' failed", event)


class LogBuffer(logging.Handler):
    """Thread-safe ring buffer of log records for the console of the user interface."""

    def __init__(self, capacity: int = 2000):
        super().__init__(level=logging.INFO)
        self.records: deque[tuple[float, int, str, str]] = deque(maxlen=capacity)
        self._lock = threading.Lock()
        self.version = 0

    def emit(self, record: logging.LogRecord) -> None:
        try:
            msg = record.getMessage()
        except Exception:
            msg = str(record.msg)
        with self._lock:
            self.records.append((time.time(), record.levelno, record.name.split(".")[-1], msg))
            self.version += 1

    def snapshot(self) -> list[tuple[float, int, str, str]]:
        with self._lock:
            return list(self.records)

    def clear(self) -> None:
        with self._lock:
            self.records.clear()
            self.version += 1


_LOG_BUFFER: LogBuffer | None = None


def install_log_buffer() -> LogBuffer:
    global _LOG_BUFFER
    if _LOG_BUFFER is None:
        _LOG_BUFFER = LogBuffer()
        logging.getLogger().addHandler(_LOG_BUFFER)
    return _LOG_BUFFER
