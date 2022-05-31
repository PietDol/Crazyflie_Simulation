# ROS IMPORTS
from std_msgs.msg import Float32MultiArray, Float32

# EAGERx IMPORTS
from eagerx_ode.engine import OdeEngine
from eagerx_pybullet.engine import PybulletEngine
from eagerx import Object, EngineNode, SpaceConverter, EngineState
from eagerx.core.specs import ObjectSpec
from eagerx.core.graph_engine import EngineGraph
import eagerx.core.register as register


class Crazyflie(Object):
    entity_id = "Crazyflie"

    @staticmethod
    @register.sensors(
        pos=Float32MultiArray, vel=Float32MultiArray, orientation=Float32MultiArray, gyroscope=Float32MultiArray,
        accelerometer=Float32MultiArray, state_estimator=Float32MultiArray,
    )
    @register.engine_states(
        pos=Float32MultiArray,
        vel=Float32MultiArray,
        orientation=Float32MultiArray,
        angular_vel=Float32MultiArray,
        lateral_friction=Float32,
        model_state=Float32MultiArray,
    )
    @register.actuators(pwm_input=Float32MultiArray, desired_thrust=Float32MultiArray,
                        desired_attitude=Float32MultiArray, commanded_thrust=Float32MultiArray,
                        commanded_attitude=Float32MultiArray)
    @register.config(urdf=None, fixed_base=True, self_collision=True, base_pos=[0, 0, 0], base_or=[0, 0, 0, 1])
    def agnostic(spec: ObjectSpec, rate):
        """Agnostic definition of the Solid"""
        # Register standard converters, space_converters, and processors
        import eagerx.converters  # noqa # pylint: disable=unused-import

        # Position
        spec.sensors.pos.rate = rate
        spec.sensors.pos.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[-999, -999, -999],
            high=[999, 999, 999],
        )
        spec.states.pos.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[-3, -3, 0],
            high=[3, 3, 0],
        )

        # Velocity
        spec.sensors.vel.rate = rate
        spec.sensors.vel.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[-100, -100, -100],
            high=[100, 100, 100],
        )
        spec.states.vel.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[0, 0, 0],
            high=[0, 0, 0],
        )

        # Orientation
        spec.sensors.orientation.rate = rate
        spec.sensors.orientation.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[-1, -1, -1, -1],
            high=[1, 1, 1, 1],
        )
        spec.states.orientation.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[0, 0, -1, -1],
            high=[0, 0, 1, 1],
        )

        # Gyroscope
        spec.sensors.gyroscope.rate = rate
        spec.sensors.gyroscope.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[-10, -10, -10],
            high=[10, 10, 10],
        )
        spec.states.angular_vel.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[0, 0, 0],
            high=[0, 0, 0],
        )

        # Accelerometer
        spec.sensors.accelerometer.rate = rate
        spec.sensors.accelerometer.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[-10, -10, -10],
            high=[10, 10, 10],
        )

        # State estimator
        spec.sensors.state_estimator.rate = rate
        spec.sensors.state_estimator.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[-10, -10, -10, -10],
            high=[10, 10, 10, 10],
        )

        # Dynamics
        spec.states.lateral_friction.space_converter = SpaceConverter.make(
            "Space_Float32",
            dtype="float32",
            low=0.1,
            high=0.5,
        )

        # Model state
        # States are: [x, y, z, x_dot, y_dot, z_dot, phi, theta, thrust_state]
        spec.states.model_state.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[-1000, 1000, 0, -100, -100, -100, -30, -30, -100000],
            high=[1000, 1000, 1000, 100, 100, 100, 30, 30, 100000],
        )

        # Actuators
        # PWM input
        spec.actuators.pwm_input.rate = rate
        spec.actuators.pwm_input.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[0.2, 0.2, 0],
            high=[0.2, 0.2, 0],
        )

        # Desired thrust
        spec.actuators.commanded_thrust.rate = rate
        spec.actuators.commanded_thrust.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[10000],
            high=[60000],
        )

        # Desired attitude
        spec.actuators.commanded_attitude.rate = rate
        spec.actuators.commanded_attitude.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[-30, -30, -30],  # [phi, theta, psi]
            high=[30, 30, 30],
        )

    @staticmethod
    @register.spec(entity_id, Object)
    def spec(
            spec: ObjectSpec,
            name: str,
            urdf: str,
            sensors=None,
            states=None,
            actuators=None,
            rate=30,
            base_pos=None,
            base_or=None,
            self_collision=True,
            fixed_base=True,
    ):
        """Object spec of Solid"""
        # Performs all the steps to fill-in the params with registered info about all functions.
        # Crazyflie.initialize_spec(spec)

        # Modify default agnostic params
        # Only allow changes to the agnostic params (rates, windows, (space)converters, etc...
        spec.config.name = name
        spec.config.sensors = sensors if sensors is not None else ["pos", "vel", "orientation", "gyroscope",
                                                                   "accelerometer", "state_estimator"]
        spec.config.states = states if states is not None else ["pos", "vel", "orientation", "angular_vel",
                                                                "model_state"]
        spec.config.actuators = actuators if actuators is not None else []  # ["external_force"]

        # Add registered agnostic params
        spec.config.urdf = urdf
        spec.config.base_pos = base_pos if base_pos else [0, 0, 0]
        spec.config.base_or = base_or if base_or else [0, 0, 0, 1]
        spec.config.self_collision = self_collision
        spec.config.fixed_base = fixed_base

        # Add agnostic implementation
        Crazyflie.agnostic(spec, rate)

    @staticmethod
    @register.engine(entity_id, PybulletEngine)
    def pybullet_engine(spec: ObjectSpec, graph: EngineGraph):
        # todo: fix the rates!!
        """Engine-specific implementation (Pybullet) of the object."""
        # Import any object specific entities for this engine
        import Crazyflie_Simulation.solid.pybullet  # noqa # pylint: disable=unused-import
        import eagerx_pybullet  # noqa # pylint: disable=unused-import

        # Set object arguments (as registered per register.engine_params(..) above the engine.add_object(...) method.
        spec.PybulletEngine.urdf = spec.config.urdf
        spec.PybulletEngine.basePosition = spec.config.base_pos
        spec.PybulletEngine.baseOrientation = spec.config.base_or
        spec.PybulletEngine.fixed_base = spec.config.fixed_base
        spec.PybulletEngine.self_collision = spec.config.self_collision

        # Create engine_states (no agnostic states defined in this case)
        spec.PybulletEngine.states.pos = EngineState.make("LinkState", mode="position")
        spec.PybulletEngine.states.vel = EngineState.make("LinkState", mode="velocity")
        spec.PybulletEngine.states.orientation = EngineState.make("LinkState", mode="orientation")
        spec.PybulletEngine.states.angular_vel = EngineState.make("LinkState", mode="angular_vel")
        spec.PybulletEngine.states.lateral_friction = EngineState.make("PbDynamics", parameter="lateralFriction")

        # Create PID engine nodes
        attitude_pid = EngineNode.make("AttitudePID", "attitude_pid", rate=spec.sensors.pos.rate, n=3)
        attitude_rate_pid = EngineNode.make("AttitudeRatePID", "attitude_rate_pid", rate=spec.sensors.pos.rate, n=3)
        power_distribution = EngineNode.make("PowerDistribution", "power_distribution", rate=spec.sensors.pos.rate, n=3)
        # Create actuator engine nodes
        # Rate=None, but we will connect it to an actuator (thus will use the rate set in the agnostic specification)
        external_force = EngineNode.make(
            "ForceController", "external_force", rate=spec.actuators.pwm_input.rate, process=2,
            mode="external_force"
        )
        # Create sensor engine nodes
        # Rate=None, but we will connect them to sensors (thus will use the rate set in the agnostic specification)
        pos = EngineNode.make("LinkSensor", "pos", rate=spec.sensors.pos.rate, process=2, mode="position")
        vel = EngineNode.make("LinkSensor", "vel", rate=spec.sensors.vel.rate, process=2, mode="velocity")
        orientation = EngineNode.make("LinkSensor", "orientation", rate=spec.sensors.orientation.rate, process=2,
                                      mode="orientation")
        gyroscope = EngineNode.make("LinkSensor", "gyroscope", rate=spec.sensors.gyroscope.rate, process=2,
                                    mode="angular_vel")
        accelerometer = EngineNode.make("AccelerometerSensor", "accelerometer", rate=spec.sensors.accelerometer.rate,
                                        process=2)
        state_estimator = EngineNode.make(
            "StateEstimator", "state_estimator", rate=spec.sensors.state_estimator.rate, n=3
        )

        # Connect all engine nodes
        graph.add([pos, vel, orientation, gyroscope, accelerometer, attitude_pid, attitude_rate_pid, power_distribution,
                   external_force, state_estimator])
        # graph.connect(source=external_force.outputs.action_applied, target=accelerometer.inputs.input_force, skip=True)
        graph.connect(source=vel.outputs.obs, target=accelerometer.inputs.input_velocity, window=2)
        graph.connect(source=pos.outputs.obs, sensor="pos")
        graph.connect(source=vel.outputs.obs, target=external_force.inputs.velocity)
        graph.connect(source=external_force.outputs.velocity_out, sensor="vel")
        # graph.connect(source=orientation.outputs.obs, sensor="orientation")
        graph.connect(source=gyroscope.outputs.obs, sensor="gyroscope")
        graph.connect(source=accelerometer.outputs.obs, sensor="accelerometer")
        graph.connect(actuator="commanded_attitude", target=attitude_pid.inputs.desired_attitude)
        graph.connect(source=gyroscope.outputs.obs, target=attitude_rate_pid.inputs.current_rate)
        graph.connect(source=attitude_pid.outputs.new_attitude_rate, target=attitude_rate_pid.inputs.desired_rate)
        graph.connect(actuator="commanded_thrust", target=power_distribution.inputs.desired_thrust)
        graph.connect(source=attitude_rate_pid.outputs.new_motor_control,
                      target=power_distribution.inputs.calculated_control)
        graph.connect(source=power_distribution.outputs.pwm_signal, target=external_force.inputs.action)
        graph.connect(source=accelerometer.outputs.obs, target=state_estimator.inputs.acceleration)
        graph.connect(source=gyroscope.outputs.obs, target=state_estimator.inputs.angular_velocity)
        graph.connect(source=orientation.outputs.obs, target=state_estimator.inputs.orientation)
        graph.connect(source=orientation.outputs.obs, target=external_force.inputs.orientation)
        # Choose between pybullet and state estimator orientation for the attitude PID
        # graph.connect(source=state_estimator.outputs.orientation_state_estimator, target=attitude_pid.inputs.current_attitude)
        graph.connect(source=state_estimator.outputs.orientation_pybullet, target=attitude_pid.inputs.current_attitude)

        graph.connect(source=state_estimator.outputs.orientation_pybullet, sensor="orientation")
        # graph.connect(source=orientation.outputs.obs, target=attitude_pid.inputs.current_attitude)

        graph.gui()
        # Check graph validity (commented out)
        # graph.is_valid(plot=True)

    @staticmethod
    @register.engine(entity_id, OdeEngine)
    def ode_engine(spec: ObjectSpec, graph: EngineGraph):
        # Import any object specific entities for this engine
        import Crazyflie_Simulation.solid.ode  # noqa # pylint: disable=unused-import

        # Set object arguments
        spec.OdeEngine.ode = "Crazyflie_Simulation.solid.eom.crazyflie_ode/crazyflie_ode"

        # Set default parameters of crazyflie ode [mass, gain, time constant]
        spec.OdeEngine.ode_params = [0.03303, 1.1094, 0.183806]

        # Create engine_states
        spec.OdeEngine.states.model_state = EngineState.make("OdeEngineState")

        # Create output engine node
        x = EngineNode.make("OdeOutput", "x", rate=spec.sensors.gyroscope.rate, process=2)

        # Create sensor engine nodes
        pos = EngineNode.make("FloatMultiArrayOutput", "pos", rate=spec.sensors.pos.rate, idx=[0, 1, 2])
        orientation = EngineNode.make("FloatMultiArrayOutput", "orientation", rate=spec.sensors.orientation.rate,
                                      idx=[6, 7])

        # Create actuator engine nodes
        action = EngineNode.make("OdeMultiInput", "crazyflie_ode", rate=spec.actuators.commanded_attitude.rate,
                                 process=2, default_action=[10000, 0, 0])

        # Connect all engine nodes
        graph.add([x, pos, orientation, action])
        # actuator
        graph.connect(actuator="commanded_attitude", target=action.inputs.commanded_attitude)
        graph.connect(actuator="commanded_thrust", target=action.inputs.commanded_thrust)

        # observation
        graph.connect(source=x.outputs.observation, target=pos.inputs.observation_array)
        graph.connect(source=x.outputs.observation, target=orientation.inputs.observation_array)

        # sensors
        graph.connect(source=pos.outputs.observation, sensor="pos")
        graph.connect(source=orientation.outputs.observation, sensor="orientation")

        # graph.gui()
