"""Thread-safe command buffer between external interfaces and the simulation loop.

External interfaces are ROS 2 callbacks and scripts. Their threads only add
commands to the queue. The simulation loop removes all commands at the start
of each frame. Only the simulation loop uses PyBullet. A command that must
send an answer has a `Reply`. The caller can wait for this reply.
"""
from __future__ import annotations

import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Optional


class Reply:
    def __init__(self):
        self._event = threading.Event()
        self.ok = False
        self.message = ""
        self.value: Any = None

    def set(self, ok: bool, message: str = "", value: Any = None) -> None:
        self.ok, self.message, self.value = ok, message, value
        self._event.set()

    def wait(self, timeout: Optional[float] = None) -> bool:
        return self._event.wait(timeout)


@dataclass
class Command:
    reply: Optional[Reply] = field(default=None, kw_only=True)


@dataclass
class JointTargetsCommand(Command):
    robot: str
    names: list[str]
    positions: list[float]      # NaN = no command
    velocities: list[float]
    efforts: list[float]


@dataclass
class CmdVelCommand(Command):
    robot: str
    linear_x: float
    angular_z: float


@dataclass
class SimControlCommand(Command):
    action: str                 # play | pause | stop | reset | step | speed
    value: float = 0.0


@dataclass
class SpawnRobotCommand(Command):
    name: str
    urdf_path: str = ""
    urdf_string: str = ""
    position: tuple = (0.0, 0.0, 0.0)
    orientation: tuple = (0.0, 0.0, 0.0, 1.0)
    allow_renaming: bool = False


@dataclass
class DeleteRobotCommand(Command):
    name: str


class CommandQueue:
    def __init__(self, maxlen: int = 10000):
        self._q: deque[Command] = deque(maxlen=maxlen)
        self._lock = threading.Lock()

    def put(self, cmd: Command) -> None:
        with self._lock:
            self._q.append(cmd)

    def drain(self) -> list[Command]:
        with self._lock:
            items = list(self._q)
            self._q.clear()
        return items

    def __len__(self) -> int:
        return len(self._q)
