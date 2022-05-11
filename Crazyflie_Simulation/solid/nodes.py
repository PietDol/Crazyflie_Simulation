import eagerx
import eagerx.converters  # Registers space converters
from eagerx.utils.utils import Msg
from std_msgs.msg import Float32, Float32MultiArray
import numpy as np


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
    def callback(self, t_n: float, desired_attitude: Msg, current_attitude: Msg):
        test = np.array([0, 0, 0])
        return dict(new_attitude_rate=Float32MultiArray(data=test))


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
        spec.inputs.current_rate.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray", [50000, 50000, 50000],
                                                                              [50000, 50000, 50000], dtype="float32")
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
    def callback(self, t_n: float, desired_rate: Msg, current_rate: Msg):
        test = np.array([10000, 5000, 2500])
        return dict(new_motor_control=Float32MultiArray(data=test))
        # pass

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
                                                                                [0],
                                                                                 [65535], dtype="float32")
        spec.inputs.calculated_control.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                                    [-32767, -32767, -32767],
                                                                                    [32767, 32767, 32767], dtype="float32")
        spec.outputs.pwm_signal.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                             [0, 0, 0, 0],
                                                                             [65535, 65535, 65535, 65535],
                                                                             dtype="float32")

    def initialize(self):
        pass

    @eagerx.register.states()
    def reset(self):
        pass

    #limit motorpowers definition
    def limitThrust(self, value):
        if (value > 65535):
            value = 65535
        elif (value < 0):
            value = 0
        return value

    @eagerx.register.inputs(desired_thrust=Float32MultiArray, calculated_control=Float32MultiArray)
    @eagerx.register.outputs(pwm_signal=Float32MultiArray)
    def callback(self, t_n: float, desired_thrust: Msg, calculated_control: Msg):
        desired_thrust.msgs[-1].data = [10000] #set input at 30000, moet nog vanuit env/actions komen

        #define variables from inputs
        calculated_control_input = calculated_control.msgs[-1].data #roll pitch yaw
        roll = calculated_control.msgs[-1].data[0]
        pitch = calculated_control.msgs[-1].data[1]
        yaw = calculated_control.msgs[-1].data[2]
        desired_thrust_input = desired_thrust.msgs[-1].data[0]
        # print(f"======\n calculated control = {calculated_control_input} \n desired thrust {desired_thrust_input}") #debug

        #limit motorpowers
        motorPower_m1 = self.limitThrust(desired_thrust_input - roll + pitch + yaw)
        motorPower_m2 = self.limitThrust(desired_thrust_input - roll - pitch - yaw)
        motorPower_m3 = self.limitThrust(desired_thrust_input + roll - pitch + yaw)
        motorPower_m4 = self.limitThrust(desired_thrust_input + roll + pitch - yaw)

        #todo: add idleThrust minimum

        new_pwm_signal = np.array([motorPower_m1, motorPower_m2, motorPower_m3, motorPower_m4]) #enginenode verwacht force
        # new_pwm_signal = np.array([3,3,0])
        return dict(pwm_signal=Float32MultiArray(data=new_pwm_signal))


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
    def callback(self, t_n: float, angular_velocity: Msg, acceleration: Msg):
        test = np.array([0, 0, 0])
        return dict(orientation=Float32MultiArray(data=test))
