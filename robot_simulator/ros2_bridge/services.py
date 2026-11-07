"""Simulation control services.

Always available (std_srvs/srv/Trigger):
    /sim/pause   /sim/resume   /sim/reset   /sim/stop   /sim/step
Simulation speed: ROS parameter `sim_speed` on the simulator node
    ros2 param set /lightweight_sim sim_speed 2.0
Standard simulation_interfaces services (if the package is installed, for
example ros-jazzy-simulation-interfaces):
    /sim/spawn_entity          simulation_interfaces/srv/SpawnEntity     (URDF uri or resource_string)
    /sim/delete_entity         simulation_interfaces/srv/DeleteEntity
    /sim/set_simulation_state  simulation_interfaces/srv/SetSimulationState
    /sim/get_simulation_state  simulation_interfaces/srv/GetSimulationState
    /sim/reset_simulation      simulation_interfaces/srv/ResetSimulation
    /sim/step_simulation       simulation_interfaces/srv/StepSimulation

Each callback adds a command to the queue. It then waits, with a time limit,
for the simulation loop to execute the command. Thus, the reply gives the
real result.
"""
from __future__ import annotations

import logging

from ..core.commands import DeleteRobotCommand, Reply, SimControlCommand, SpawnRobotCommand
from .ros_node import SIM_INTERFACES_AVAILABLE

log = logging.getLogger(__name__)
REPLY_TIMEOUT = 10.0


def _run(commands, cmd) -> Reply:
    cmd.reply = Reply()
    commands.put(cmd)
    if not cmd.reply.wait(REPLY_TIMEOUT):
        cmd.reply.ok, cmd.reply.message = False, "timed out waiting for the simulation loop"
    return cmd.reply


def create_services(node, commands, get_state) -> list:
    from std_srvs.srv import Trigger

    services = []

    def trigger(action: str):
        def cb(_req, resp):
            r = _run(commands, SimControlCommand(action))
            resp.success, resp.message = r.ok, r.message or action
            return resp
        return cb

    for name, action in (("pause", "pause"), ("resume", "play"), ("reset", "reset"), ("stop", "stop"),
                         ("step", "step")):
        services.append(node.create_service(Trigger, f"/sim/{name}", trigger(action)))

    if not SIM_INTERFACES_AVAILABLE:
        log.warning("simulation_interfaces not installed: spawn/delete services are unavailable "
                    "(install ros-$ROS_DISTRO-simulation-interfaces)")
        return services

    from simulation_interfaces.msg import Result, SimulationState
    from simulation_interfaces.srv import (DeleteEntity, GetSimulationState, ResetSimulation, SetSimulationState,
                                           SpawnEntity, StepSimulation)

    def result(resp, reply, fail_code=Result.RESULT_OPERATION_FAILED):
        resp.result.result = Result.RESULT_OK if reply.ok else fail_code
        resp.result.error_message = reply.message
        return resp

    def spawn(req, resp):
        uri = req.uri
        if uri.startswith("file://"):
            uri = uri[7:]
        if uri and not uri.lower().endswith(".urdf"):
            resp.result.result = SpawnEntity.Response.UNSUPPORTED_FORMAT
            resp.result.error_message = "only URDF files are supported"
            return resp
        if not uri and not req.resource_string:
            resp.result.result = SpawnEntity.Response.NO_RESOURCE
            resp.result.error_message = "uri and resource_string are both empty"
            return resp
        p, o = req.initial_pose.pose.position, req.initial_pose.pose.orientation
        reply = _run(commands, SpawnRobotCommand(req.name, uri, req.resource_string, (p.x, p.y, p.z),
                                                 (o.x, o.y, o.z, o.w), req.allow_renaming))
        if reply.ok:
            resp.entity_name = reply.value or ""
            resp.result.result = Result.RESULT_OK
        else:
            resp.result.result = (SpawnEntity.Response.NAME_NOT_UNIQUE if "already taken" in reply.message
                                  else SpawnEntity.Response.RESOURCE_PARSE_ERROR)
            resp.result.error_message = reply.message
        return resp

    def delete(req, resp):
        return result(resp, _run(commands, DeleteRobotCommand(req.entity)), Result.RESULT_NOT_FOUND)

    def set_state(req, resp):
        mapping = {SimulationState.STATE_PLAYING: "play", SimulationState.STATE_PAUSED: "pause",
                   SimulationState.STATE_STOPPED: "stop", SimulationState.STATE_QUITTING: "quit"}
        action = mapping.get(req.state.state)
        if action is None:
            resp.result.result = Result.RESULT_FEATURE_UNSUPPORTED
            resp.result.error_message = f"state {req.state.state} not supported"
            return resp
        current = get_state()
        if current == action or (current == "playing" and action == "play") or \
                (current == "paused" and action == "pause") or (current == "stopped" and action == "stop"):
            resp.result.result = SetSimulationState.Response.ALREADY_IN_TARGET_STATE
            return resp
        if action == "pause" and current == "stopped":
            resp.result.result = SetSimulationState.Response.INCORRECT_TRANSITION
            resp.result.error_message = "cannot pause a stopped simulation"
            return resp
        return result(resp, _run(commands, SimControlCommand(action)))

    def get_state_cb(_req, resp):
        s = get_state()
        resp.state.state = {"playing": SimulationState.STATE_PLAYING, "paused": SimulationState.STATE_PAUSED,
                            "stopped": SimulationState.STATE_STOPPED}[s]
        resp.result.result = Result.RESULT_OK
        return resp

    def reset(_req, resp):
        return result(resp, _run(commands, SimControlCommand("reset")))

    def step(req, resp):
        if get_state() == "playing":
            resp.result.result = Result.RESULT_OPERATION_FAILED
            resp.result.error_message = "simulation must be paused to step"
            return resp
        return result(resp, _run(commands, SimControlCommand("step", float(max(1, req.steps)))))

    services += [
        node.create_service(SpawnEntity, "/sim/spawn_entity", spawn),
        node.create_service(DeleteEntity, "/sim/delete_entity", delete),
        node.create_service(SetSimulationState, "/sim/set_simulation_state", set_state),
        node.create_service(GetSimulationState, "/sim/get_simulation_state", get_state_cb),
        node.create_service(ResetSimulation, "/sim/reset_simulation", reset),
        node.create_service(StepSimulation, "/sim/step_simulation", step),
    ]
    return services
