import eagerx
import eagerx.converters  # Registers space converters
from eagerx.utils.utils import Msg
from std_msgs.msg import Float32, Float32MultiArray


class AttitudePID(eagerx.Node):
    @staticmethod
    @eagerx.register.spec("AttitudePID", eagerx.Node)
    def spec(
            spec,
            name: str,
            rate: float,
            n: int,
    ):
        """
        MovingAverage filter
        :param spec: Not provided by user.
        :param name: Node name
        :param rate: Rate at which callback is called.
        :param n: Window size of the moving average
        :return:
        """
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(MovingAverageFilter)

        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = eagerx.process.ENVIRONMENT
        spec.config.inputs = ["desiredAttitude", "actualAttitude"]
        spec.config.outputs = ["desiredAttitudeRate"]

        # Custom node params
        # START EXERCISE 1.1

        # START EXERCISE 1.1

        # Add space converters
        spec.inputs.desiredAttitude.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray", [-1, -1, -1, -1], [1, 1, 1, 1],
                                                                           dtype="float32")
        spec.inputs.actualAttitude.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray", [-1, -1, -1, -1], [1, 1, 1, 1],
                                                                           dtype="float32")
        spec.outputs.desiredAttitudeRate.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray", [-1, -1, -1, -1], [1, 1, 1, 1],
                                                                           dtype="float32")
    # START EXERCISE 1.2
    def initialize(self):
        pass

    # END EXERCISE 1.2

    @eagerx.register.states()
    def reset(self):
        # START EXERCISE 1.3
        pass
        # END EXERCISE 1.3

    @eagerx.register.inputs(signal=Float32)
    @eagerx.register.outputs(filtered=Float32MultiArray)
    def callback(self, t_n: float, signal: Msg):
        data = signal.msgs[-1].data

        # START EXERCISE 1.4
        filtered_data = data
        # END EXERCISE 1.4

        return dict(filtered=Float32MultiArray(data=[filtered_data]))
