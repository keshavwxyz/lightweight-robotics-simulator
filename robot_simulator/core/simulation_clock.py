"""Fixed-timestep accumulator.

The accumulator adds the wall-clock time, multiplied by the simulation speed.
It then uses this time in full physics steps. Thus, the frame rate of the
renderer does not change the physics timestep. After a slow frame, the
simulation does more steps, but not more than `max_steps`. If more time
accumulates, the accumulator discards it and the simulation runs slower than
real time.
"""
from __future__ import annotations

from collections import deque


class SimulationClock:
    def __init__(self, timestep: float, max_steps: int = 64):
        self.timestep = timestep
        self.max_steps = max_steps
        self.accumulator = 0.0
        self.dropped_time = 0.0
        self._rtf_window: deque[tuple[float, float]] = deque(maxlen=120)  # (wall_dt, sim_dt)

    def advance(self, wall_dt: float, speed: float) -> int:
        """Return the number of physics steps for `wall_dt` seconds of wall-clock time."""
        if wall_dt < 0:
            wall_dt = 0.0
        self.accumulator += wall_dt * speed
        n = int(self.accumulator / self.timestep + 1e-9)
        if n > self.max_steps:
            self.dropped_time += (n - self.max_steps) * self.timestep
            n = self.max_steps
            self.accumulator = 0.0
        else:
            self.accumulator -= n * self.timestep
        self._rtf_window.append((wall_dt, n * self.timestep))
        return n

    def reset(self) -> None:
        self.accumulator = 0.0
        self._rtf_window.clear()

    @property
    def real_time_factor(self) -> float:
        wall = sum(w for w, _ in self._rtf_window)
        sim = sum(s for _, s in self._rtf_window)
        return sim / wall if wall > 1e-9 else 0.0
