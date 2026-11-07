"""Fixed-timestep scheduler: the rendering rate does not change the physics."""
import random

import pytest

from robot_simulator.core.commands import CmdVelCommand, JointTargetsCommand, Reply, SimControlCommand
from robot_simulator.core.robot_instance import RobotConfig
from robot_simulator.core.simulation_clock import SimulationClock
from robot_simulator.motors.motor_base import MotorConfig

from .conftest import ARM_URDF, diff_drive_config


@pytest.mark.parametrize("fps", [24, 60, 144, 500])
def test_physics_steps_depend_on_wall_time_not_frame_rate(fps):
    clock = SimulationClock(1 / 240, max_steps=1000)
    steps = sum(clock.advance(1 / fps, 1.0) for _ in range(fps * 2))   # 2 s of wall time
    assert steps == pytest.approx(480, abs=1)


def test_irregular_frame_times_and_speed():
    clock = SimulationClock(1 / 240, max_steps=1000)
    rng = random.Random(1)
    wall = 0.0
    steps = 0
    while wall < 3.0:
        dt = rng.uniform(0.001, 0.05)
        wall += dt
        steps += clock.advance(dt, 2.0)
    assert steps * (1 / 240) == pytest.approx(wall * 2.0, abs=1 / 240)


def test_max_steps_cap_drops_time_instead_of_spiralling():
    clock = SimulationClock(1 / 240, max_steps=10)
    assert clock.advance(1.0, 1.0) == 10
    assert clock.dropped_time > 0.9
    assert clock.accumulator == 0.0


def test_rendering_rate_does_not_change_physics_result(sim):
    """The same wall-clock time at 30 FPS and at 200 FPS gives the same simulated state."""
    results = []
    for fps in (30, 200):
        for name in list(sim.robots):
            sim.remove_robot(name)
        sim.reset()
        r = sim.spawn_robot(RobotConfig(urdf_path=ARM_URDF, fixed_base=True))
        m = r.assign_motor(MotorConfig(joint="joint1", type="rotational", mode="velocity", max_effort=80,
                                       max_velocity=2))
        m.set_velocity(0.5)
        sim.play()
        for _ in range(fps):
            sim.update(1.0 / fps)
        results.append((sim.engine.step_count, float(r.physics.joint_states().position[0])))
        sim.pause()
    assert results[0][0] == results[1][0] == 240
    assert results[0][1] == pytest.approx(results[1][1], abs=1e-9)


def test_commands_from_other_threads_are_applied_by_the_loop(sim):
    r = sim.spawn_robot(diff_drive_config())
    arm = sim.spawn_robot(RobotConfig(urdf_path=ARM_URDF, fixed_base=True, position=(3, 0, 0)))
    arm.assign_motor(MotorConfig(joint="joint1", type="rotational", max_effort=80, max_velocity=2))
    sim.commands.put(CmdVelCommand(r.name, 0.3, 0.0))
    sim.commands.put(JointTargetsCommand(arm.name, ["joint1"], [0.5], [], []))
    reply = Reply()
    sim.commands.put(SimControlCommand("speed", 2.0, reply=reply))
    sim.process_commands()
    assert reply.ok and sim.speed == 2.0
    assert r.drive.v == 0.3
    assert arm.motors.get("joint1").config.target_position == 0.5
    bad = Reply()
    sim.commands.put(JointTargetsCommand(arm.name, ["joint2"], [0.5], [], [], reply=bad))
    sim.process_commands()
    assert not bad.ok and "no motor" in bad.message
