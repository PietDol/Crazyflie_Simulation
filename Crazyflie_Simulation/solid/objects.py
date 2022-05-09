# ROS IMPORTS
from std_msgs.msg import Float32MultiArray, Float32

# EAGERx IMPORTS
from eagerx_pybullet.bridge import PybulletBridge
from eagerx import Object, EngineNode, SpaceConverter, EngineState
from eagerx.core.specs import ObjectSpec
from eagerx.core.graph_engine import EngineGraph
import eagerx.core.register as register


class Solid(Object):
    entity_id = "Solid"

    @staticmethod
    @register.sensors(
        pos=Float32MultiArray, vel=Float32MultiArray, orientation=Float32MultiArray, angular_vel=Float32MultiArray
    )
    @register.engine_states(
        pos=Float32MultiArray,
        vel=Float32MultiArray,
        orientation=Float32MultiArray,
        angular_vel=Float32MultiArray,
        lateral_friction=Float32,
    )
    @register.actuators(external_force=Float32MultiArray)
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

        # Angular velocity
        spec.sensors.angular_vel.rate = rate
        spec.sensors.angular_vel.space_converter = SpaceConverter.make(
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

        # Dynamics
        spec.states.lateral_friction.space_converter = SpaceConverter.make(
            "Space_Float32",
            dtype="float32",
            low=0.1,
            high=0.5,
        )

        # Actuators
        spec.actuators.external_force.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[0.2, 0.2, 0],
            high=[0.2, 0.2, 0],
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
        Solid.initialize_spec(spec)

        # Modify default agnostic params
        # Only allow changes to the agnostic params (rates, windows, (space)converters, etc...
        spec.config.name = name
        spec.config.sensors = sensors if sensors is not None else ["pos", "vel", "orientation", "angular_vel"]
        spec.config.states = states if states is not None else ["pos", "vel", "orientation", "angular_vel"]
        spec.config.actuators = actuators if actuators is not None else []  #["external_force"]

        # Add registered agnostic params
        spec.config.urdf = urdf
        spec.config.base_pos = base_pos if base_pos else [0, 0, 0]
        spec.config.base_or = base_or if base_or else [0, 0, 0, 1]
        spec.config.self_collision = self_collision
        spec.config.fixed_base = fixed_base

        # Add agnostic implementation
        Solid.agnostic(spec, rate)

    @staticmethod
    @register.bridge(entity_id, PybulletBridge)
    def pybullet_bridge(spec: ObjectSpec, graph: EngineGraph):
        """Engine-specific implementation (Pybullet) of the object."""
        # Import any object specific entities for this bridge
        import Crazyflie_Simulation.solid.pybullet  # noqa # pylint: disable=unused-import
        import eagerx_pybullet  # noqa # pylint: disable=unused-import

        # Set object arguments (as registered per register.bridge_params(..) above the bridge.add_object(...) method.
        spec.PybulletBridge.urdf = spec.config.urdf
        spec.PybulletBridge.basePosition = spec.config.base_pos
        spec.PybulletBridge.baseOrientation = spec.config.base_or
        spec.PybulletBridge.fixed_base = spec.config.fixed_base
        spec.PybulletBridge.self_collision = spec.config.self_collision

        # Create engine_states (no agnostic states defined in this case)
        spec.PybulletBridge.states.pos = EngineState.make("LinkState", mode="position")
        spec.PybulletBridge.states.vel = EngineState.make("LinkState", mode="velocity")
        spec.PybulletBridge.states.orientation = EngineState.make("LinkState", mode="orientation")
        spec.PybulletBridge.states.angular_vel = EngineState.make("LinkState", mode="angular_vel")
        spec.PybulletBridge.states.lateral_friction = EngineState.make("PbDynamics", parameter="lateralFriction")

        # Create sensor engine nodes
        # Rate=None, but we will connect them to sensors (thus will use the rate set in the agnostic specification)
        pos = EngineNode.make("LinkSensor", "pos", rate=spec.sensors.pos.rate, process=2, mode="position")
        vel = EngineNode.make("LinkSensor", "vel", rate=spec.sensors.vel.rate, process=2, mode="velocity")
        orientation = EngineNode.make(
            "LinkSensor", "orientation", rate=spec.sensors.orientation.rate, process=2, mode="orientation"
        )
        angular_vel = EngineNode.make(
            "LinkSensor", "angular_vel", rate=spec.sensors.angular_vel.rate, process=2, mode="angular_vel"
        )

        # Create actuator engine nodes
        # Rate=None, but we will connect it to an actuator (thus will use the rate set in the agnostic specification)
        external_force = EngineNode.make(
            "ForceController", "external_force", rate=spec.actuators.external_force.rate, process=2, mode="external_force"
        )
        # Connect all engine nodes
        graph.add([pos, vel, orientation, angular_vel, external_force])
        graph.connect(source=pos.outputs.obs, sensor="pos")
        graph.connect(source=vel.outputs.obs, sensor="vel")
        graph.connect(source=orientation.outputs.obs, sensor="orientation")
        graph.connect(source=angular_vel.outputs.obs, sensor="angular_vel")
        graph.connect(actuator="external_force", target=external_force.inputs.action)

        # Check graph validity (commented out)
        # graph.is_valid(plot=True)


class Solid1(Object):
    entity_id = "Solid1"

    @staticmethod
    @register.sensors(
        pos=Float32MultiArray, vel=Float32MultiArray, orientation=Float32MultiArray, angular_vel=Float32MultiArray
    )
    @register.engine_states(
        pos=Float32MultiArray,
        vel=Float32MultiArray,
        orientation=Float32MultiArray,
        angular_vel=Float32MultiArray,
        lateral_friction=Float32,
    )
    # @register.actuators(external_force=Float32MultiArray)
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
            low=[-1, -1, 0],
            high=[1, 1, 0],
        )

        # Velocity
        spec.sensors.vel.rate = rate
        spec.sensors.vel.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[-10, -10, -10],
            high=[10, 10, 10],
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

        # Angular velocity
        spec.sensors.angular_vel.rate = rate
        spec.sensors.angular_vel.space_converter = SpaceConverter.make(
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

        # Dynamics
        spec.states.lateral_friction.space_converter = SpaceConverter.make(
            "Space_Float32",
            dtype="float32",
            low=0.1,
            high=0.5,
        )

        # Actuators
        # spec.actuators.external_force.space_converter = SpaceConverter.make(
        #     "Space_Float32MultiArray",
        #     dtype="float32",
        #     low=[0, 0, -10],
        #     high=[0, 0, 10],
        # )

    @staticmethod
    @register.spec(entity_id, Object)
    def spec(
        spec: ObjectSpec,
        name: str,
        urdf: str,
        sensors=None,
        states=None,
        # actuators=None,
        rate=30,
        base_pos=None,
        base_or=None,
        self_collision=True,
        fixed_base=True,
    ):
        """Object spec of Solid"""
        # Performs all the steps to fill-in the params with registered info about all functions.
        Solid1.initialize_spec(spec)

        # Modify default agnostic params
        # Only allow changes to the agnostic params (rates, windows, (space)converters, etc...
        spec.config.name = name
        spec.config.sensors = sensors if sensors is not None else ["pos", "vel", "orientation", "angular_vel"]
        spec.config.states = states if states is not None else ["pos", "vel", "orientation", "angular_vel"]
        # spec.config.actuators = actuators if actuators is not None else ["external_force"]

        # Add registered agnostic params
        spec.config.urdf = urdf
        spec.config.base_pos = base_pos if base_pos else [0, 0, 0]
        spec.config.base_or = base_or if base_or else [0, 0, 0, 1]
        spec.config.self_collision = self_collision
        spec.config.fixed_base = fixed_base

        # Add agnostic implementation
        Solid1.agnostic(spec, rate)

    @staticmethod
    @register.bridge(entity_id, PybulletBridge)
    def pybullet_bridge(spec: ObjectSpec, graph: EngineGraph):
        """Engine-specific implementation (Pybullet) of the object."""
        # Import any object specific entities for this bridge
        import eagerx_interbotix.solid.pybullet  # noqa # pylint: disable=unused-import
        import eagerx_pybullet  # noqa # pylint: disable=unused-import

        # Set object arguments (as registered per register.bridge_params(..) above the bridge.add_object(...) method.
        spec.PybulletBridge.urdf = spec.config.urdf
        spec.PybulletBridge.basePosition = spec.config.base_pos
        spec.PybulletBridge.baseOrientation = spec.config.base_or
        spec.PybulletBridge.fixed_base = spec.config.fixed_base
        spec.PybulletBridge.self_collision = spec.config.self_collision

        # Create engine_states (no agnostic states defined in this case)
        spec.PybulletBridge.states.pos = EngineState.make("LinkState", mode="position")
        spec.PybulletBridge.states.vel = EngineState.make("LinkState", mode="velocity")
        spec.PybulletBridge.states.orientation = EngineState.make("LinkState", mode="orientation")
        spec.PybulletBridge.states.angular_vel = EngineState.make("LinkState", mode="angular_vel")
        spec.PybulletBridge.states.lateral_friction = EngineState.make("PbDynamics", parameter="lateralFriction")

        # Create sensor engine nodes
        # Rate=None, but we will connect them to sensors (thus will use the rate set in the agnostic specification)
        pos = EngineNode.make("LinkSensor", "pos", rate=spec.sensors.pos.rate, process=2, mode="position")
        vel = EngineNode.make("LinkSensor", "vel", rate=spec.sensors.vel.rate, process=2, mode="velocity")
        orientation = EngineNode.make(
            "LinkSensor", "orientation", rate=spec.sensors.orientation.rate, process=2, mode="orientation"
        )
        angular_vel = EngineNode.make(
            "LinkSensor", "angular_vel", rate=spec.sensors.angular_vel.rate, process=2, mode="angular_vel"
        )

        # Create actuator engine nodes
        # Rate=None, but we will connect it to an actuator (thus will use the rate set in the agnostic specification)
        # external_force = EngineNode.make(
        #     "ForceController", "external_force", rate=spec.actuators.external_force.rate, process=2, mode="external_force"
        # )

        # Connect all engine nodes
        # graph.add([pos, vel, orientation, angular_vel, external_force])
        graph.connect(source=pos.outputs.obs, sensor="pos")
        graph.connect(source=vel.outputs.obs, sensor="vel")
        graph.connect(source=orientation.outputs.obs, sensor="orientation")
        graph.connect(source=angular_vel.outputs.obs, sensor="angular_vel")
        # graph.connect(actuator="external_force", target=external_force.inputs.action)

        # Check graph validity (commented out)
        # graph.is_valid(plot=True)
