# EAGERx imports
from eagerx.wrappers.flatten import Flatten
from eagerx.core.env import EagerxEnv
from eagerx.core.graph import Graph
import eagerx.nodes  # Registers butterworth_filter # noqa # pylint: disable=unused-import
import eagerx_pybullet  # Registers PybulletEngine # noqa # pylint: disable=unused-import
import Crazyflie_Simulation  # Registers objects # noqa # pylint: disable=unused-import
import eagerx_reality  # Registers Engine # noqa # pylint: disable=unused-import
import Crazyflie_Simulation.solid.nodes
from Crazyflie_Simulation.solid.pid import PID

# Other
import numpy as np
import stable_baselines3 as sb
from datetime import datetime
import os

NAME = "varyGoal_term_noExcl"
LOG_DIR = os.path.dirname(
    Crazyflie_Simulation.__file__) + f"/../logs/{NAME}_{datetime.today().strftime('%Y-%m-%d-%H%M')}"

# todo: check the windows and rates

if __name__ == "__main__":
    eagerx.initialize("eagerx_core", anonymous=True, log_level=eagerx.log.WARN)

    # Define rate
    real_reset = False
    rate = 500 #220?
    safe_rate = 500
    max_steps = 300

    # Initialize empty graph
    graph = Graph.create()

    # Create solid object
    urdf_path = os.path.dirname(Crazyflie_Simulation.__file__) + "/solid/assets/"
    crazyflie = eagerx.Object.make(
        "Crazyflie", "crazyflie", urdf=urdf_path + "cf2x.urdf", rate=rate, sensors=["pos", "vel", "orientation", "gyroscope", "accelerometer"], actuators=["pwm_input"],
        base_pos=[0, 0, 1], fixed_base=False,
        states=["pos", "vel", "orientation", "angular_vel"]
    )
    crazyflie.sensors.pos.space_converter.low = [0, -1, 0]
    crazyflie.sensors.pos.space_converter.high = [1, 1, 0.15]
    crazyflie.states.lateral_friction.space_converter.low = 0.4
    crazyflie.states.lateral_friction.space_converter.high = 0.1
    graph.add(crazyflie)

    # Add attitude PID node to graph
    attitude_pid = eagerx.Node.make(
        "AttitudePID", "attitude_pid", rate=rate, n=3
    )
    graph.add(attitude_pid)

    # Add attitude rate PID node to graph
    attitude_rate_pid = eagerx.Node.make(
        "AttitudeRatePID", "attitude_rate_pid", rate=rate, n=3
    )
    graph.add(attitude_rate_pid)

    # Add power distribution
    power_distribution = eagerx.Node.make(
        "PowerDistribution", "power_distribution", rate=rate, n=3
    )
    graph.add(power_distribution)

    # Add state estimator
    state_estimator = eagerx.Node.make(
        "StateEstimator", "state_estimator", rate=rate, n=3
    )
    graph.add(state_estimator)

    # Connecting observations
    graph.connect(source=crazyflie.sensors.orientation, observation="orientation")
    graph.connect(source=crazyflie.sensors.pos, observation="position")

    # Connecting actions
    # graph.connect(action="external_force", target=solid.actuators.external_force)
    graph.connect(action="desired_attitude", target=attitude_pid.inputs.desired_attitude)
    graph.connect(action="desired_thrust", target=power_distribution.inputs.desired_thrust)
    graph.connect(source=attitude_pid.outputs.new_attitude_rate, target=attitude_rate_pid.inputs.desired_rate)
    graph.connect(source=attitude_rate_pid.outputs.new_motor_control, target=power_distribution.inputs.calculated_control)
    graph.connect(source=power_distribution.outputs.pwm_signal, target=crazyflie.actuators.pwm_input)
    graph.connect(source=crazyflie.sensors.gyroscope, target=state_estimator.inputs.angular_velocity)
    graph.connect(source=crazyflie.sensors.gyroscope, target=attitude_rate_pid.inputs.current_rate)
    graph.connect(source=crazyflie.sensors.accelerometer, target=state_estimator.inputs.acceleration)
    graph.connect(source=crazyflie.sensors.orientation, target=state_estimator.inputs.orientation)
    graph.connect(source=state_estimator.outputs.orientation, target=attitude_pid.inputs.current_attitude)

    # Create reset node
    if real_reset:
        # Connect target state we are resetting
        graph.connect(action="pwm_input", target=crazyflie.actuators)
        # Connect joint output to safety filter

    # Show in the gui
    # graph.gui()

    # Define engines
    # engine = Engine.make("RealEngine", rate=rate, sync=True, process=process.NEW_PROCESS)
    engine = eagerx.Engine.make("PybulletEngine", rate=safe_rate, gui=True, egl=True, sync=True, real_time_factor=0.0)
                                # process=eagerx.process.ENVIRONMENT)  # delete process to run faster, this is useful for debugger

    # Define step function
    def step_fn(prev_obs, obs, action, steps):
        # Set info:
        info = dict()
        # Calculate reward

        can = obs["position"][0]
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
        done = False
        return obs, rwd, done, info


    # Define reset function
    def reset_fn(env):
        states = env.state_space.sample()

        # Set orientation
        states["crazyflie/orientation"] = np.array([0, 0, 0, 1])

        states["crazyflie/pos"] = np.array([0, 0, 1])
        return states


    # Initialize Environment
    env = EagerxEnv(name="rx", rate=rate, graph=graph, engine=engine, step_fn=step_fn, reset_fn=reset_fn, exclude=[])

    # First train in simulation
    env.render("human")

    # Evaluate
    for eps in range(5000):
        print(f"Episode {eps}")
        _, done = env.reset(), False
        desired_altitude = 2
        while not done:
            desired_thrust_pid = PID(kp=10000, ki=50, kd=2500000, rate=rate) #kp 2500 ki 0.2 kd 10000

            action = env.action_space.sample()
            action["desired_attitude"][0] = 0           # Roll
            action["desired_attitude"][1] = 0           # Pitch
            action["desired_attitude"][2] = 0           # Yaw
            try:
                action["desired_thrust"][0] = desired_thrust_pid.next_action(current=obs["position"][0][2], desired=desired_altitude)
            except:
                # print("fucked") # debug
                action["desired_thrust"][0] = desired_thrust_pid.next_action(current=0, desired=desired_altitude)
            # action["desired_thrust"][0] = 36100
            obs, reward, done, info = env.step(action)
            rgb = env.render("rgb_array")
            # print("Orientation:")
            # print(obs["orientation"])
            # print("Position:")
            # print(obs["position"])

