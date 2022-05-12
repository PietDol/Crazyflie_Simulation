import eagerx
import eagerx.converters  # Registers space converters
from eagerx.utils.utils import Msg
from std_msgs.msg import Float32, Float32MultiArray
import numpy as np
from Crazyflie_Simulation.solid.pid import PID


# todo: implement functions for the nodes
# todo: check boundaries for all classes
# todo: integration limit pids


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
        self.attitude_pid_yaw = PID(kp=6, ki=1, kd=0.35, rate=self.rate)
        self.attitude_pid_pitch = PID(kp=6, ki=3, kd=0, rate=self.rate)
        self.attitude_pid_roll = PID(kp=6, ki=3, kd=0, rate=self.rate)

    @eagerx.register.states()
    def reset(self):
        self.attitude_pid_yaw.reset()
        self.attitude_pid_pitch.reset()
        self.attitude_pid_roll.reset()

    @eagerx.register.inputs(desired_attitude=Float32MultiArray, current_attitude=Float32MultiArray)
    @eagerx.register.outputs(new_attitude_rate=Float32MultiArray)
    def callback(self, t_n: float, desired_attitude: Msg, current_attitude: Msg):
        current_attitude = current_attitude.msgs[-1].data
        desired_attitude = desired_attitude.msgs[-1].data

        next_yaw = self.attitude_pid_yaw.next_action(current=current_attitude[0], desired=desired_attitude[0])
        next_pitch = self.attitude_pid_pitch.next_action(current=current_attitude[1], desired=desired_attitude[1])
        next_roll = self.attitude_pid_roll.next_action(current=current_attitude[2], desired=desired_attitude[2])
        next_action = np.array([next_roll, next_pitch, next_yaw])
        return dict(new_attitude_rate=Float32MultiArray(data=next_action))


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
        spec.inputs.current_rate.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                              [50000, 50000, 50000],
                                                                              [50000, 50000, 50000], dtype="float32")
        spec.outputs.new_motor_control.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                                    [-32767, -32767, -32767],
                                                                                    [32767, 32767, 32767],
                                                                                    dtype="float32")

    def initialize(self):
        self.attitude_rate_pid_yaw = PID(kp=120, ki=16.7, kd=0, rate=self.rate)
        self.attitude_rate_pid_pitch = PID(kp=250, ki=500, kd=2.5, rate=self.rate)
        self.attitude_rate_pid_roll = PID(kp=250, ki=500, kd=2.5, rate=self.rate)

    @eagerx.register.states()
    def reset(self):
        self.attitude_rate_pid_yaw.reset()
        self.attitude_rate_pid_pitch.reset()
        self.attitude_rate_pid_roll.reset()

    @eagerx.register.inputs(desired_rate=Float32MultiArray, current_rate=Float32MultiArray)
    @eagerx.register.outputs(new_motor_control=Float32MultiArray)
    def callback(self, t_n: float, desired_rate: Msg, current_rate: Msg):
        current_attitude_rate = current_rate.msgs[-1].data
        desired_attitude_rate = desired_rate.msgs[-1].data

        next_yaw_rate = self.attitude_rate_pid_yaw.next_action(current=current_attitude_rate[0],
                                                               desired=desired_attitude_rate[0])
        next_pitch_rate = self.attitude_rate_pid_pitch.next_action(current=current_attitude_rate[1],
                                                                   desired=desired_attitude_rate[1])
        next_roll_rate = self.attitude_rate_pid_roll.next_action(current=current_attitude_rate[2],
                                                                 desired=desired_attitude_rate[2])
        next_action = np.array([next_roll_rate, next_pitch_rate, next_yaw_rate])
        return dict(new_motor_control=Float32MultiArray(data=next_action))


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
                                                                                    [32767, 32767, 32767],
                                                                                    dtype="float32")
        spec.outputs.pwm_signal.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                             [0, 0, 0, 0],
                                                                             [65535, 65535, 65535, 65535],
                                                                             dtype="float32")

    def initialize(self):
        pass

    @eagerx.register.states()
    def reset(self):
        pass

    # limit motorpowers definition
    def limitThrust(self, value):
        if value > 65535:
            value = 65535
        elif value < 0:
            value = 0
        return value

    # limit minimum idle Thrust definition
    def limitIdleThrust(self, value, minimum):
        if (value < minimum):
            value = minimum
        return value

    @eagerx.register.inputs(desired_thrust=Float32MultiArray, calculated_control=Float32MultiArray)
    @eagerx.register.outputs(pwm_signal=Float32MultiArray)
    def callback(self, t_n: float, desired_thrust: Msg, calculated_control: Msg):
        desired_thrust.msgs[-1].data  # set input at 30000, moet nog vanuit env/actions komen

        # define variables from inputs
        calculated_control_input = calculated_control.msgs[-1].data  # roll pitch yaw
        roll = calculated_control.msgs[-1].data[0]
        pitch = calculated_control.msgs[-1].data[1]
        yaw = calculated_control.msgs[-1].data[2]
        desired_thrust_input = desired_thrust.msgs[-1].data[0]
        # print(f"======\n calculated control = {calculated_control_input} \n desired thrust {desired_thrust_input}") #debug

        # limit motorpowers
        motorPower_m1 = self.limitThrust(desired_thrust_input - roll + pitch + yaw)
        motorPower_m2 = self.limitThrust(desired_thrust_input - roll - pitch - yaw)
        motorPower_m3 = self.limitThrust(desired_thrust_input + roll - pitch + yaw)
        motorPower_m4 = self.limitThrust(desired_thrust_input + roll + pitch - yaw)

        # limit minimum idle Thrust
        minimumIdleThrust = 0  # PWM signal, default = 0
        motorPower_m1 = self.limitIdleThrust(motorPower_m1, minimumIdleThrust)
        motorPower_m2 = self.limitIdleThrust(motorPower_m2, minimumIdleThrust)
        motorPower_m3 = self.limitIdleThrust(motorPower_m3, minimumIdleThrust)
        motorPower_m4 = self.limitIdleThrust(motorPower_m4, minimumIdleThrust)

        # print(motorPower_m4) # debug

        new_pwm_signal = np.array(
            [motorPower_m1, motorPower_m2, motorPower_m3, motorPower_m4])  # enginenode verwacht force
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
        spec.config.inputs = ["angular_velocity", "acceleration", "orientation"]
        spec.config.outputs = ["orientation"]

        # Add space converters
        spec.inputs.angular_velocity.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                                  [0, 0, 0],
                                                                                  [3, 3, 3], dtype="float32")
        spec.inputs.acceleration.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                              [0, 0, 0],
                                                                              [3, 3, 3], dtype="float32")
        spec.inputs.orientation.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
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

    @eagerx.register.inputs(angular_velocity=Float32MultiArray, acceleration=Float32MultiArray,
                            orientation=Float32MultiArray)
    @eagerx.register.outputs(orientation=Float32MultiArray)
    def callback(self, t_n: float, angular_velocity: Msg, acceleration: Msg, orientation: Msg):
        attitude = orientation.msgs[-1].data
        return dict(orientation=Float32MultiArray(data=attitude))

# class AltitudePID(eagerx.Node):
#     @staticmethod
#     @eagerx.register.spec("AltitudePID", eagerx.Node)
#     def spec(
#             spec,
#             name: str,
#             rate: float,
#             n: int,
#     ):
#         # Performs all the steps to fill-in the params with registered info about all functions.
#         spec.initialize(AltitudePID)
#
#         # Modify default node params
#         spec.config.name = name
#         spec.config.rate = rate
#         spec.config.process = eagerx.process.ENVIRONMENT
#         spec.config.inputs = ["estimated_position", "estimated_velocity", "desired_position"]
#         spec.config.outputs = ["desired_attitude", "desired_thrust"]
#
#         # Add space converters
#         spec.inputs.estimated_position.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray", [0, 0, 0],
#                                                                               [3, 3, 3], dtype="float32")
#         spec.inputs.estimated_velocity.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray", [0, 0, 0],
#                                                                               [5, 5, 5], dtype="float32")
#         spec.inputs.desired_position.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray", [0, 0, 0],
#                                                                               [3, 3, 3], dtype="float32")
#         spec.outputs.desired_attitude.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
#                                                                                     [-32767, -32767, -32767],
#                                                                                     [32767, 32767, 32767],
#                                                                                     dtype="float32")
#         spec.outputs.desired_thrust.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
#                                                                                     [-32767, -32767, -32767],
#                                                                                     [32767, 32767, 32767],
#                                                                                     dtype="float32")
#
#     def initialize(self):
#         # self.attitude_rate_pid_yaw = PID(kp=120, ki=16.7, kd=0, rate=self.rate)
#         # self.attitude_rate_pid_pitch = PID(kp=250, ki=500, kd=2.5, rate=self.rate)
#         # self.attitude_rate_pid_roll = PID(kp=250, ki=500, kd=2.5, rate=self.rate)
#         pass
#
#     @eagerx.register.states()
#     def reset(self):
#         # self.attitude_rate_pid_yaw.reset()
#         # self.attitude_rate_pid_pitch.reset()
#         # self.attitude_rate_pid_roll.reset()
#         pass
#
#     @eagerx.register.inputs(estimated_position=Float32MultiArray, estimated_velocity=Float32MultiArray, desired_position=Float32MultiArray)
#     @eagerx.register.outputs(desired_attitude=Float32MultiArray, desired_thrust=Float32MultiArray)
#     def callback(self, t_n: float, estimated_position: Msg):
#         # current_attitude_rate = current_rate.msgs[-1].data
#         # desired_attitude_rate = desired_rate.msgs[-1].data
#         #
#         # next_yaw_rate = self.attitude_rate_pid_yaw.next_action(current=current_attitude_rate[0],
#         #                                                        desired=desired_attitude_rate[0])
#         # next_pitch_rate = self.attitude_rate_pid_pitch.next_action(current=current_attitude_rate[1],
#         #                                                            desired=desired_attitude_rate[1])
#         # next_roll_rate = self.attitude_rate_pid_roll.next_action(current=current_attitude_rate[2],
#         #                                                          desired=desired_attitude_rate[2])
#         # next_action = np.array([next_roll_rate, next_pitch_rate, next_yaw_rate])
#         # return dict(new_motor_control=Float32MultiArray(data=next_action))
#         pass
