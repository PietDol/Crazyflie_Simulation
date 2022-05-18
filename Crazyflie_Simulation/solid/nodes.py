import eagerx
import eagerx.converters  # Registers space converters
from eagerx.utils.utils import Msg
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import Image
import numpy as np
from Crazyflie_Simulation.solid.pid import PID
import pybullet
import cv2


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
        spec.inputs.desired_attitude.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                                  [-30, -30, -30],
                                                                                  [30, 30, 30], dtype="float32")
        spec.inputs.current_attitude.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                                  [-1, -1, -1, -1],
                                                                                  [1, 1, 1, 1], dtype="float32")
        spec.outputs.new_attitude_rate.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                                    [-90, -90, -90],
                                                                                    [90, 90, 90], dtype="float32")

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
        # Current attitude is a quaternion
        current_attitude = current_attitude.msgs[-1].data
        # Desired attitude is in degrees
        desired_attitude = desired_attitude.msgs[-1].data
        # Current attitude from quaternion to euler angles (RPY)
        current_attitude_euler = pybullet.getEulerFromQuaternion(current_attitude)
        # Current attitude from radians to degrees
        current_attitude_deg = np.zeros(3)
        for idx, ang in enumerate(current_attitude_euler):
            current_attitude_deg[idx] = ang * 180 / np.pi

        # print(f"Desired attitude: {desired_attitude[1]}")
        # print(f"Current attitude: {current_attitude_deg[1]}")

        next_roll = self.attitude_pid_roll.next_action(current=current_attitude_deg[0], desired=desired_attitude[0])
        # todo: (-) minus sign (necessary to make it work)
        next_pitch = self.attitude_pid_pitch.next_action(current=-current_attitude_deg[1], desired=desired_attitude[1])
        next_yaw = self.attitude_pid_yaw.next_action(current=current_attitude_deg[2], desired=desired_attitude[2])
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
        spec.inputs.desired_rate.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                              [-90, -90, -90],
                                                                              [90, 90, 90], dtype="float32")
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
        current_attitude_rate = current_rate.msgs[-1].data  # (vx, vy, vz) Roll, pitch, yaw
        desired_attitude_rate = desired_rate.msgs[-1].data
        # print(f"Current rate: {current_attitude_rate[1]}")
        # print(f"Desired rate: {desired_attitude_rate[1]}")
        # print("-" * 50)
        next_roll_rate = self.attitude_rate_pid_roll.next_action(current=current_attitude_rate[0],
                                                                 desired=desired_attitude_rate[0])
        next_pitch_rate = self.attitude_rate_pid_pitch.next_action(current=current_attitude_rate[1],
                                                                   desired=desired_attitude_rate[1])
        next_yaw_rate = self.attitude_rate_pid_yaw.next_action(current=current_attitude_rate[2],
                                                               desired=desired_attitude_rate[2])

        next_action = np.array([next_roll_rate, next_pitch_rate, next_yaw_rate])
        # print(next_action)
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


class MakePicture(eagerx.Node):
    @staticmethod
    @eagerx.register.spec("MakePicture", eagerx.Node)
    def spec(
            spec,
            name: str,
            rate: float,
    ):
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(MakePicture)

        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = eagerx.process.ENVIRONMENT
        spec.config.inputs = ["position", "orientation"]
        spec.config.outputs = ["image"]

        # Add space converters
        spec.inputs.position.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                                  [0, 0, 0],
                                                                                  [3, 3, 3], dtype="float32")
        spec.inputs.orientation.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                              [0, 0, 0],
                                                                              [3, 3, 3], dtype="float32")
        # spec.outputs.image.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
        #                                                                       [0, 0, 0],
        #                                                                       [3, 3, 3], dtype="float32")

    def initialize(self):
        pass

    @eagerx.register.states()
    def reset(self):
        pass

    @eagerx.register.inputs(position=Float32MultiArray, orientation=Float32MultiArray)
    @eagerx.register.outputs(image=Image)
    def callback(self, t_n: float, position: Msg, orientation: Msg):
        height = 880
        width = 880
        offset = 40 # offset of the picture from the sides
        pos_x, pos_y, pos_z = position.msgs[-1].data[0], position.msgs[-1].data[1], position.msgs[-1].data[2]

        euler_orientation = pybullet.getEulerFromQuaternion(orientation.msgs[-1].data)
        roll, pitch, yaw = euler_orientation[0], euler_orientation[1], euler_orientation[2]

        img = np.zeros((height, width, 3), np.uint8)
        img[:,:] = (255,255,255)

        for i in range(9): # add coordinate system to the rendered picture y axis
            y_axis = np.linspace(0, 4, 9)
            length = 10
            x = offset
            text_height = 4
            y = height - i*100 - offset
            img = cv2.line(img, (x, y), (x - length, y), (0,0,0), 1) # make markers on y-axis
            img = cv2.putText(img, str(y_axis[i]), (5, y + text_height), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,0))

        for i in range(9):  # add coordinate system to the rendered picture x axis
            x_axis = np.linspace(2, -2, 9)
            length = 10
            x = width - i * 100 - offset
            text_height = 4
            y = height - offset
            img = cv2.line(img, (x, height - offset), (x, height - offset + length), (0, 0, 0), 1)  # make markers on x-axis
            img = cv2.putText(img, str(x_axis[i]), (x - text_height*4, y + 25), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,0))

        #  create border
        img = cv2.rectangle(img, (offset, offset), (height - offset, width - offset), (0,0,0), 1)
        arm_length = 0.028*5

        def plot_x(img):
            """"Changes the plot so that you can see from the x axis side"""
            z_correction = arm_length*np.sin(-pitch)
            x_correction = arm_length*np.cos(-pitch)
            # print(f'pitch is: {-pitch*180/np.pi} degrees')
            img = cv2.circle(img,  (int((pos_x + x_correction)*200)//1 + width//2 , height - int((pos_z+ z_correction)*200//1) - offset), 5, (255, 0, 0), -1)
            img = cv2.circle(img,  (int((pos_x - x_correction)*200)//1 + width//2 , height - int((pos_z - z_correction)*200//1) - offset), 5, (255, 0, 0), -1)
            img = cv2.line(img, (int((pos_x + x_correction)*200)//1 + width//2 , height - int((pos_z+ z_correction)*200//1) - offset),
                           (int((pos_x - x_correction)*200)//1 + width//2 , height - int((pos_z - z_correction)*200//1) - offset), (255, 0, 0), 2)
            return img

        def plot_y(img):
            """"Changes the plot so that you can see from the y axis side"""
            z_correction = arm_length*np.sin(roll)
            y_correction = arm_length*np.cos(roll)
            img = cv2.circle(img,  (int((pos_y + y_correction)*200)//1 + width//2 , height - int((pos_z+ z_correction)*200//1) - offset), 5, (255, 0, 0), -1)
            img = cv2.circle(img,  (int((pos_y - y_correction)*200)//1 + width//2 , height - int((pos_z - z_correction)*200//1) - offset), 5, (255, 0, 0), -1)
            img = cv2.line(img, (int((pos_y + y_correction)*200)//1 + width//2 , height - int((pos_z+ z_correction)*200//1) - offset),
                           (int((pos_y - y_correction)*200)//1 + width//2 , height - int((pos_z - z_correction)*200//1) - offset), (255, 0, 0), 2)
            return img

        img = plot_x(img) # uncomment this line to show plot from x_side
        # img = plot_y(img) # uncomment this line to show plot from y_side
        data = img.tobytes("C")
        msg = Image(data=data, height=height, width=width, encoding="bgr8", step=3 * width)
        return dict(image=msg)


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
