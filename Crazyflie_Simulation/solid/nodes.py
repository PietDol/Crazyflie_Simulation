import eagerx
import eagerx.converters  # Registers space converters
from eagerx.utils.utils import Msg
from std_msgs.msg import Float32, Float32MultiArray


# todo: implement functions for the nodes
# todo: check boundaries for all classes


class AttitudePID(eagerx.Node):
    @staticmethod
    @eagerx.register.spec("AttitudePID", eagerx.Node)
    def spec(
            spec,
            name: str,
            rate: float,
            n: int,
    ):
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(AttitudePID)

        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = eagerx.process.ENVIRONMENT
        spec.config.inputs = ["desired_attitude", "current_attitude"]
        spec.config.outputs = ["new_attitude_rate"]

        # Add space converters
        spec.inputs.desired_attitude.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray", [0, 0, 0],
                                                                                  [3, 3, 3], dtype="float32")
        spec.inputs.current_attitude.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray", [0, 0, 0],
                                                                                  [3, 3, 3], dtype="float32")
        spec.outputs.new_attitude_rate.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                                    [0, 0, 0],
                                                                                    [3, 3, 3], dtype="float32")

    def initialize(self):
        pass

    @eagerx.register.states()
    def reset(self):
        pass

    @eagerx.register.inputs(desired_attitude=Float32MultiArray, current_attitude=Float32MultiArray)
    @eagerx.register.outputs(new_attitude_rate=Float32MultiArray)
    def callback(self, t_n: float, signal: Msg):
        pass


class AttitudeRatePID(eagerx.Node):
    @staticmethod
    @eagerx.register.spec("AttitudeRatePID", eagerx.Node)
    def spec(
            spec,
            name: str,
            rate: float,
            n: int,
    ):
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(AttitudeRatePID)

        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = eagerx.process.ENVIRONMENT
        spec.config.inputs = ["desired_rate", "current_rate"]
        spec.config.outputs = ["new_motor_control"]

        # Add space converters
        spec.inputs.desired_rate.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray", [0, 0, 0],
                                                                              [3, 3, 3], dtype="float32")
        spec.inputs.current_rate.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray", [0, 0, 0],
                                                                              [3, 3, 3], dtype="float32")
        spec.outputs.new_motor_control.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                                    [-32767, -32767, -32767],
                                                                                    [32767, 32767, 32767],
                                                                                    dtype="float32")

    def initialize(self):
        pass

    @eagerx.register.states()
    def reset(self):
        pass

    @eagerx.register.inputs(desired_rate=Float32MultiArray, current_rate=Float32MultiArray)
    @eagerx.register.outputs(new_motor_control=Float32MultiArray)
    def callback(self, t_n: float, signal: Msg):
        pass


class PowerDistribution(eagerx.Node):
    @staticmethod
    @eagerx.register.spec("PowerDistribution", eagerx.Node)
    def spec(
            spec,
            name: str,
            rate: float,
            n: int,
    ):
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(PowerDistribution)

        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = eagerx.process.ENVIRONMENT
        spec.config.inputs = ["desired_thrust", "calculated_control"]
        spec.config.outputs = ["pwm_signal"]

        # Add space converters
        spec.inputs.desired_thrust.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                                [[0, 0, 0], [0, 0, 0], [0, 0, 0],
                                                                                 [0, 0, 0]],
                                                                                [[3, 3, 3], [3, 3, 3], [3, 3, 3],
                                                                                 [3, 3, 3]], dtype="float32")
        spec.inputs.calculated_control.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                                    [[0, 0, 0], [0, 0, 0], [0, 0, 0],
                                                                                     [0, 0, 0]],
                                                                                    [[3, 3, 3], [3, 3, 3], [3, 3, 3],
                                                                                     [3, 3, 3]], dtype="float32")
        spec.outputs.pwm_signal.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                             [0, 0, 0, 0],
                                                                             [65535, 65535, 65535, 65535],
                                                                             dtype="float32")

    def initialize(self):
        pass

    @eagerx.register.states()
    def reset(self):
        pass

    @eagerx.register.inputs(desired_thrust=Float32MultiArray, calculated_control=Float32MultiArray)
    @eagerx.register.outputs(pwm_signal=Float32MultiArray)
    def callback(self, t_n: float, signal: Msg):
        pass


class StateEstimator(eagerx.Node):
    @staticmethod
    @eagerx.register.spec("StateEstimator", eagerx.Node)
    def spec(
            spec,
            name: str,
            rate: float,
            n: int,
    ):
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(StateEstimator)

        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = eagerx.process.ENVIRONMENT
        spec.config.inputs = ["angular_velocity", "acceleration"]
        spec.config.outputs = ["orientation"]

        # Add space converters
        spec.inputs.angular_velocity.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                                [0, 0, 0],
                                                                                [3, 3, 3], dtype="float32")
        spec.inputs.acceleration.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                                [0, 0, 0],
                                                                                [3, 3, 3], dtype="float32")
        spec.outputs.orientation.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                                [0, 0, 0],
                                                                                [3, 3, 3], dtype="float32")

    def initialize(self):
        pass

    @eagerx.register.states()
    def reset(self):
        pass

    @eagerx.register.inputs(angular_velocity=Float32MultiArray, acceleration=Float32MultiArray)
    @eagerx.register.outputs(orientation=Float32MultiArray)
    def callback(self, t_n: float, signal: Msg):
        pass
