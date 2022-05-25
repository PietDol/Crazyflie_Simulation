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
from Crazyflie_Simulation.solid.log import Log

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
    rate = 220  # 220?
    safe_rate = 220
    max_steps = 1000

    # Initialize empty graph
    graph = Graph.create()

    # Set URDF path
    urdf_path = os.path.dirname(Crazyflie_Simulation.__file__) + "/solid/assets/"

    # Create PID node
    pid_height = eagerx.Node.make("HeightPID", "pid_height", rate=48)
    validate_pid = eagerx.Node.make("ValidatePID", "validate_pid", rate=48)

    # - - - - - - - CHOOSE ENGINE MODE HERE - - - - - - -
    # engine_mode = "Pybullet"  # choose between Pybullet and Ode (bridge select)

    if engine_mode == "Pybullet":
        # - - - - - - - PYBULLET START - - - - - - -
        # Define Crazyflie Object
        crazyflie = eagerx.Object.make(
            "Crazyflie", "crazyflie", urdf=urdf_path + "cf2x.urdf", rate=rate,
            sensors=["pos", "vel", "orientation", "gyroscope", "accelerometer"],
            actuators=["commanded_thrust", "commanded_attitude"],
            base_pos=[0, 0, 1], fixed_base=False,
            states=["pos", "vel", "orientation", "angular_vel"]
        )
        # set sensor spaceconverter values
        crazyflie.sensors.pos.space_converter.low = [0, -1, 0]
        crazyflie.sensors.pos.space_converter.high = [1, 1, 0.15]
        # set states spaceconverter values
        crazyflie.states.lateral_friction.space_converter.low = 0.4
        crazyflie.states.lateral_friction.space_converter.high = 0.1

        # Define engine
        # engine = Engine.make("RealEngine", rate=rate, sync=True, process=process.NEW_PROCESS)
        engine = eagerx.Engine.make("PybulletEngine", rate=safe_rate, gui=True, egl=True, sync=True,
                                    real_time_factor=0.0)
        # - - - - - - - PYBULLET END - - - - - - -
    elif engine_mode == "Ode":
        # - - - - - - - ODE START - - - - - - -
        # Define Crazyflie Object
        crazyflie = eagerx.Object.make(
            "Crazyflie", "crazyflie", urdf=urdf_path + "cf2x.urdf", rate=rate,
            sensors=["pos", "orientation"],
            actuators=["commanded_thrust", 'commanded_attitude'],
            base_pos=[0, 0, 1], fixed_base=False,
            states=["model_state"]
        )

        # Define engine
        engine = eagerx.Engine.make("OdeEngine", rate=safe_rate, sync=True, real_time_factor=0.0)
        # - - - - - - - ODE END - - - - - - -
    else:
        raise "Wrong engine_mode selected. Please choose between Pybullet and Ode"

    # Add picture making node and EDIT CONFIGURATION
    make_picture = eagerx.Node.make(
        "MakePicture", "make_picture", rate,
        save_render_image=save_render_image,
        saveToPreviousRender=saveToPreviousRender,
        renderColor=renderColor, #choose between black, red, blue
        axisToPlot=axisToPlot, #choose between x, y
        max_steps=max_steps,
        engine_mode=engine_mode,
    )
    # Create agnostic graph
    graph.add([make_picture, crazyflie, validate_pid])
    # graph.add(crazyflie)
    # graph.add(pid_height)
    # graph.add(validate_pid)

    # Connect Crazyflie inputs
    # graph.connect(action="desired_attitude", target=crazyflie.actuators.commanded_attitude)
    graph.connect(action="desired_position", target=validate_pid.inputs.desired_position)
    # graph.connect(action="desired_thrust", target=crazyflie.actuators.commanded_thrust)
    # graph.connect(action="desired_height", target=pid_height.inputs.desired_height)
    # graph.connect(source=pid_height.outputs.new_action, target=crazyflie.actuators.commanded_thrust)
    graph.connect(source=validate_pid.outputs.new_thrust, target=crazyflie.actuators.commanded_thrust)
    graph.connect(source=validate_pid.outputs.new_attitude, target=crazyflie.actuators.commanded_attitude)

    # Connect Crazyflie outputs
    graph.connect(source=crazyflie.sensors.orientation, observation="orientation")
    graph.connect(source=crazyflie.sensors.pos, observation="position")
    # graph.connect(source=crazyflie.sensors.pos, target=pid_height.inputs.current_height)
    graph.connect(source=crazyflie.sensors.pos, target=validate_pid.inputs.current_position)

    # Connect picture making node
    graph.connect(source=crazyflie.sensors.orientation, target=make_picture.inputs.orientation)
    graph.connect(source=crazyflie.sensors.pos, target=make_picture.inputs.position)
    graph.render(source=make_picture.outputs.image, rate=rate)

    # Create reset node
    if real_reset:
        # Connect target state we are resetting
        graph.connect(action="pwm_input", target=crazyflie.actuators)

    # Show in the gui
    # graph.gui()
    # graph.is_valid(plot=True)

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
            done = False
        # done = done | (np.linalg.norm(goal - can) < 0.1 and can[2] < 0.05)  # Can has not fallen down & within threshold.
        # done = False
        return obs, rwd, done, info


    # Define reset function
    def reset_fn(env):
        states = env.state_space.sample()

        if engine_mode == "Pybullet":
            # Reset states for Pybullet engine
            states["crazyflie/orientation"] = np.array([0, 0, 0, 1])
            states["crazyflie/pos"] = np.array([0, 0, 1])
        elif engine_mode == "Ode":
            # States are: [x, y, z, x_dot, y_dot, z_dot, phi, theta, thrust_state]
            states["crazyflie/model_state"] = np.array([0, 0, 1, 0, 0, 0, 0, 0, 0])
        else:
            raise "Wrong engine_mode selected. Please choose between Pybullet and Ode"

        return states

    # Initialize Environment
    env = EagerxEnv(name="rx", rate=rate, graph=graph, engine=engine, step_fn=step_fn, reset_fn=reset_fn, exclude=[])

    # First train in simulation
    # env.render("human")

    _, done = env.reset(), False
    # Evaluate

    # desired_altitude = 2
    # desired_thrust_pid = PID(kp=0.2, ki=0.0001, kd=0.4, rate=rate)  # kp 10000 ki 50 kd 2500000
    while not done:
        action = env.action_space.sample()
        # action["desired_attitude"][0] = 0  # Roll
        # action["desired_attitude"][1] = 0  # Pitch
        # action["desired_attitude"][2] = 0  # Yaw
        # action["desired_height"] = np.array([desired_altitude])
        action["desired_position"] = np.array([0, 0, 6])
        obs, reward, done, info = env.step(action)
        rgb = env.render("rgb_array")

        log.add_data(position=obs["position"][0], orientation=obs["orientation"][0], run_id=run_id, rate=rate)
