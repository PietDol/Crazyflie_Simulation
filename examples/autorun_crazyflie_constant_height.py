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

def runEagerX(engine_mode, save_render_image, saveToPreviousRender, renderColor, axisToPlot):
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

        # - - - - - - - CHOOSE ENGINE MODE HERE - - - - - - -
        # engine_mode = "Pybullet" # choose between Pybullet and Ode (bridge select)

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

        # Add picture making node
        make_picture = eagerx.Node.make(
            "MakePicture", "make_picture", rate, save_render_image=save_render_image,
            saveToPreviousRender=saveToPreviousRender, renderColor=renderColor, axisToPlot=axisToPlot,
            max_steps=max_steps,
        )

        # Create agnostic graph
        graph.add(make_picture)
        graph.add(crazyflie)
        # Connect Crazyflie inputs
        graph.connect(action="desired_attitude", target=crazyflie.actuators.commanded_attitude)
        graph.connect(action="desired_thrust", target=crazyflie.actuators.commanded_thrust)
        # Connect Crazyflie outputs
        graph.connect(source=crazyflie.sensors.orientation, observation="orientation")
        graph.connect(source=crazyflie.sensors.pos, observation="position")
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
            rwd = 0

            # Determine done flag
            if steps > max_steps:  # Max steps reached
                done = True
                info["TimeLimit.truncated"] = True
            else:
                done = False

            # done = done | (np.linalg.norm(goal - can) < 0.1 and can[2] < 0.05)  # Can has not fallen down & within threshold.
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
        env = EagerxEnv(name="rx", rate=rate, graph=graph, engine=engine, step_fn=step_fn, reset_fn=reset_fn,
                        exclude=[])

        # First train in simulation
        # env.render("human")

        # Evaluate
        _, done = env.reset(), False
        desired_altitude = 1
        while not done:
            desired_thrust_pid = PID(kp=10000, ki=50, kd=2500000, rate=rate)  # kp 10000 ki 50 kd 2500000

            action = env.action_space.sample()
            action["desired_attitude"][0] = 10  # Roll
            action["desired_attitude"][1] = 0  # Pitch
            action["desired_attitude"][2] = 0  # Yaw
            try:
                action["desired_thrust"][0] = desired_thrust_pid.next_action(current=obs["position"][0][2],
                                                                             desired=desired_altitude)
            except:
                action["desired_thrust"][0] = desired_thrust_pid.next_action(current=0, desired=desired_altitude)
            action["desired_thrust"][0] = np.clip(action["desired_thrust"][0], 10000, 60000)

            # Set RENDER vars according to definition

            obs, reward, done, info = env.step(action)
            rgb = env.render("rgb_array")


# run EagerX Pybullet followed by EagerX Ode
# args:     mode (Pybullet, Ode),
#           save_render_image (True, False),
#           saveToPreviousRender (True, False),
#           render color (black, red, blue)
#           axisToPlot (x, y)
axisToPlot = "y"
runEagerX("Pybullet",  # first run
          save_render_image=True,
          saveToPreviousRender=False,
          renderColor="black",
          axisToPlot=axisToPlot)

runEagerX("Ode",  # second run
          save_render_image=True,
          saveToPreviousRender=True,
          renderColor="red",
          axisToPlot=axisToPlot)
