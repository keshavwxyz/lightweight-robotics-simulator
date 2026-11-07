"""Lightweight Robotics Simulator: command-line entry point.

    python -m robot_simulator.main                         # GUI
    python -m robot_simulator.main --project path/to/project.json
    python -m robot_simulator.main --urdf robot.urdf --play
    python -m robot_simulator.main --headless --project p.json   # no window
"""
from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

from .core.configuration import AppConfig

DEFAULT_CONFIG = Path(__file__).with_name("config.json")


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description="Lightweight robotics simulator (PyBullet + OpenGL + ROS 2)")
    ap.add_argument("--project", help="project.json (or project directory) to open")
    ap.add_argument("--urdf", action="append", default=[], help="URDF file to import (repeatable)")
    ap.add_argument("--config", default=str(DEFAULT_CONFIG), help="application config.json")
    ap.add_argument("--no-ros", action="store_true", help="do not start the ROS 2 bridge")
    ap.add_argument("--play", action="store_true", help="start the simulation immediately")
    ap.add_argument("--headless", action="store_true", help="run without a window")
    ap.add_argument("--duration", type=float, default=None, help="headless: stop after this simulation time [s]")
    ap.add_argument("--no-cameras", action="store_true", help="headless: do not create an OpenGL context")
    ap.add_argument("--screenshot", help=argparse.SUPPRESS)  # for tests: save one frame, then stop
    ap.add_argument("-v", "--verbose", action="store_true")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    # Diagnostics: `kill -USR1 <pid>` writes the stack of each thread to stderr.
    import faulthandler
    import signal
    faulthandler.register(signal.SIGUSR1, all_threads=True)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s", datefmt="%H:%M:%S")
    logging.getLogger("OpenGL").setLevel(logging.WARNING)
    config = AppConfig.load(args.config)
    if args.headless:
        from .core.application import HeadlessRunner
        runner = HeadlessRunner(config, args.project, tuple(args.urdf), enable_ros=not args.no_ros,
                                cameras=not args.no_cameras)
        runner.run(args.duration)
        return 0
    from .core.application import Application
    app = Application(config, args.project, tuple(args.urdf), enable_ros=False if args.no_ros else None,
                      autoplay=args.play)
    if args.screenshot:
        app.screenshot_path = args.screenshot
    return app.run()


if __name__ == "__main__":
    sys.exit(main())
