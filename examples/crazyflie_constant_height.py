# EAGERx imports
from eagerx.wrappers.flatten import Flatten
from eagerx.core.env import EagerxEnv
from eagerx.core.graph import Graph
import eagerx.nodes  # Registers butterworth_filter # noqa # pylint: disable=unused-import
import eagerx_pybullet  # Registers PybulletBridge # noqa # pylint: disable=unused-import
import Crazyflie_Simulation  # Registers objects # noqa # pylint: disable=unused-import
import eagerx_reality  # Registers bridge # noqa # pylint: disable=unused-import

# Other
import numpy as np
import stable_baselines3 as sb
from datetime import datetime
import os

NAME = "varyGoal_term_noExcl"
LOG_DIR = os.path.dirname(
    Crazyflie_Simulation.__file__) + f"/../logs/{NAME}_{datetime.today().strftime('%Y-%m-%d-%H%M')}"

# todo: increase friction coefficient (seems to glide too much)
# todo: velocity control
# todo: Increase the penalty on velocity
# todo: switch goal with object position
# todo: normalize actions/observations

if __name__ == "__main__":
    eagerx.initialize("eagerx_core", anonymous=True, log_level=eagerx.log.WARN)

    # Define rate
    real_reset = False
    rate = 20
    safe_rate = 20
    max_steps = 300

    # Initialize empty graph
    graph = Graph.create()

    # Create solid object
    urdf_path = os.path.dirname(Crazyflie_Simulation.__file__) + "/solid/assets/"
    solid = eagerx.Object.make(
        "Solid", "solid", urdf=urdf_path + "cf2x.urdf", rate=rate, sensors=["pos"], actuators=["external_force"],
        base_pos=[0, 0, 1], fixed_base=False,
        states=["pos", "vel", "orientation", "angular_vel", "lateral_friction"]
    )
    solid.sensors.pos.space_converter.low = [0, -1, 0]
    solid.sensors.pos.space_converter.high = [1, 1, 0.15]
    solid.states.lateral_friction.space_converter.low = 0.4
    solid.states.lateral_friction.space_converter.high = 0.1
    graph.add(solid)

    # Connecting observations
    graph.connect(source=solid.sensors.pos, observation="solid")

    # Connecting actions
    graph.connect(action="external_force", target=solid.actuators.external_force)

    # Create reset node
    if real_reset:
        # Disconnect simulation-specific connections

        # Connect target state we are resetting
        graph.connect(action="external_force", target=solid.actuators)
        # Connect joint output to safety filter

        # Connect inputs to determine reset status

    # Show in the gui
    # graph.gui()

    # Define bridges
    # bridge = Bridge.make("RealBridge", rate=rate, sync=True, process=process.NEW_PROCESS)
    bridge = eagerx.Bridge.make("PybulletBridge", rate=safe_rate, gui=True, egl=True, sync=True, real_time_factor=0.0,
                                process=eagerx.process.ENVIRONMENT)  # delete process to run faster, this is useful for debugger

    # Define step function
    def step_fn(prev_obs, obs, action, steps):
        # Set info:
        info = dict()
        # Calculate reward
        can = obs["solid"][0]
        # Penalize distance of the end-effector to the object
        rwd = 0
        # Determine done flag
        if steps > max_steps:  # Max steps reached
            done = True
            info["TimeLimit.truncated"] = True
        else:
            done = False | (np.linalg.norm(can[:2]) > 1.0)  # Can is out of reach
            if done:
                rwd = -50
        # done = done | (np.linalg.norm(goal - can) < 0.1 and can[2] < 0.05)  # Can has not fallen down & within threshold.
        return obs, rwd, done, info


    # Define reset function
    def reset_fn(env):
        states = env.state_space.sample()

        # Set orientation
        states["solid/orientation"] = np.array([0, 0, 0, 1])

        states["solid/pos"] = np.array([0, 0, 0])
        return states


    # Initialize Environment
    env = EagerxEnv(name="rx", rate=rate, graph=graph, bridge=bridge, step_fn=step_fn, reset_fn=reset_fn, exclude=[])

    # First train in simulation
    env.render("human")

    # Evaluate
    for eps in range(5000):
        print(f"Episode {eps}")
        _, done = env.reset(), False
        while not done:
            action = env.action_space.sample()
            obs, reward, done, info = env.step(action)
            rgb = env.render("rgb_array")
