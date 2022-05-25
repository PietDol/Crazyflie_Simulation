import eagerx
import eagerx.converters  # Registers space converters
from eagerx.utils.utils import Msg
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import Image
import numpy as np
from Crazyflie_Simulation.solid.pid import PID
import pybullet
import cv2
from typing import Optional, List


# todo: implement functions for the nodes
# todo: check boundaries for all classes
# todo: integration limit pids

class MakePicture(eagerx.Node):
    @staticmethod
    @eagerx.register.spec("MakePicture", eagerx.Node)
    def spec(
            spec,
            name: str,
            rate: float,
            save_render_image: bool,
            saveToPreviousRender: bool,
            renderColor: str,
            axisToPlot: str,
            max_steps: int,
    ):
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(MakePicture)

        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = eagerx.process.ENVIRONMENT
        spec.config.inputs = ["position", "orientation"]
        spec.config.outputs = ["image"]

        spec.config.save_render_image = save_render_image if save_render_image else False
        spec.config.saveToPreviousRender = saveToPreviousRender if saveToPreviousRender else False
        spec.config.renderColor = renderColor if renderColor else "black"
        spec.config.axisToPlot = axisToPlot if axisToPlot else "x"
        spec.config.max_steps = max_steps if max_steps else 500

        # Add space converters
        spec.inputs.position.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                          [0, 0, 0],
                                                                          [3, 3, 3], dtype="float32")
        spec.inputs.orientation.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                             [0, 0, 0],
                                                                             [3, 3, 3], dtype="float32")
    def initialize(self, save_render_image, saveToPreviousRender, renderColor, axisToPlot, max_steps):
        # Change render settings here
        self.height = 880                       # set render height [px]
        self.width = 880                        # set render width [px]
        self.offset = 40                        # offset of the picture from the sides [px]
        self.timestep = 0.1                     # set timestep for render [s]
        self.length = 10
        self.text_height = 4
        self.font = cv2.FONT_HERSHEY_PLAIN
        self.xrange = [-2, 2]                   # range of x (x in 2D image = to right) [m]
        self.yrange = [0, 4]                    # range of y (y in 2D image = up) [m]
        self.amountOfDivisions = 9              # amount of divisions on axis, def=9

        # set drone arm lengths
        self.arm_length = 0.028 * 4

        # Settings which are set when creating the MakePicture Node
        self.save_render_image = save_render_image # enable or disable render from 2D plot
        self.saveToPreviousRender = saveToPreviousRender # saves render on top of last render
        self.renderColor = renderColor # blue, red or black for now
        self.axis_to_plot = axisToPlot # 'x' or 'y' right now
        self.sample_length = (0.95*max_steps)/(self.rate) - self.timestep #ensure it is rendered at least once, def=2

        # AUTO INITIALIZATIONS
        # init initial image
        self.modulus_prev = 1000
        if self.saveToPreviousRender == True:
            self.final_image = cv2.imread(f'../Crazyflie_Simulation/solid/Rendering/Images/final_image.png')
        else:
            self.final_image = 0

        # init render color
        if self.renderColor == "blue":
            self.renderColor = [255, 0, 0]
        elif self.renderColor == "red":
            self.renderColor = [0, 0, 255]
        elif self.renderColor == "black":
            self.renderColor = [0, 0, 0]
        else:
            self.renderColor = [255, 0, 0]

        # init axis for render
        self.y_axis = np.linspace(self.yrange[0], self.yrange[1], self.amountOfDivisions)
        for idx, y in enumerate(self.y_axis):
            self.y_axis[idx] = '%.2f'%(y)
        self.x_axis = np.linspace(self.xrange[1], self.xrange[0], self.amountOfDivisions)
        for idx, x in enumerate(self.x_axis):
            self.x_axis[idx] = '%.2f'%(x)

        # calculate scaling and offsets
        self.scaling_x = (self.width-2*self.offset)/(max(self.x_axis)-min(self.x_axis)) # set scaling per division
        self.scaling_y = (self.height-2*self.offset)/(max(self.y_axis)-min(self.y_axis))
        self.offset_left = abs(min(self.x_axis)/(max(self.x_axis)-min(self.x_axis))*(self.width-2*self.offset)) # set offset
        self.offset_top= abs(max(self.y_axis)/(max(self.y_axis)-min(self.y_axis))*(self.height-2*self.offset))


    @eagerx.register.states()
    def reset(self):
        pass

    @eagerx.register.inputs(position=Float32MultiArray, orientation=Float32MultiArray)
    @eagerx.register.outputs(image=Image)
    def callback(self, t_n: float, position: Msg, orientation: Msg):
        pos_x, pos_y, pos_z = position.msgs[-1].data[0], position.msgs[-1].data[1], position.msgs[-1].data[2]

        if len(orientation.msgs[-1].data) == 4:
            euler_orientation = pybullet.getEulerFromQuaternion(orientation.msgs[-1].data)
        else:
            euler_orientation = np.array(orientation.msgs[-1].data) * np.pi / 180
        roll, pitch, yaw = euler_orientation[0], -euler_orientation[1], euler_orientation[2]

        img = np.zeros((self.height, self.width, 3), np.uint8)
        img[:, :] = (255, 255, 255)

        for i in range(self.amountOfDivisions):  # add coordinate system to the rendered picture y axis
            y_axis = self.y_axis
            x = self.offset
            y = int(self.height - i * ((self.height-2*self.offset)/(self.amountOfDivisions-1)) - self.offset)
            img = cv2.line(img, (x, y), (x - self.length, y), (0, 0, 0), 1)  # make markers on y-axis
            img = cv2.putText(img, str(y_axis[i]), (5, y + self.text_height), self.font, 1, (0, 0, 0))

        for i in range(self.amountOfDivisions):  # add coordinate system to the rendered picture x axis
            x_axis = self.x_axis
            x = int(self.width - i * ((self.width-2*self.offset)/(self.amountOfDivisions-1)) - self.offset)
            y = self.height - self.offset
            img = cv2.line(img, (x, self.height - self.offset), (x, self.height - self.offset + self.length), (0, 0, 0),
                           1)  # make markers on x-axis
            img = cv2.putText(img, str(x_axis[i]), (x - self.text_height * 4, y + 25), self.font, 1, (0, 0, 0))

        #  create border
        img = cv2.rectangle(img, (self.offset, self.offset), (self.height - self.offset, self.width - self.offset),
                            (0, 0, 0), 1)

        # give the sampled image an axis like for the render image
        if type(self.final_image) is int:
            self.final_image = img

        def plot_x(img):
            """"Changes the plot so that you can see from the x axis side"""
            z_correction = self.arm_length * np.sin(-pitch)
            x_correction = self.arm_length * np.cos(-pitch)
            # print(f'pitch is: {-pitch*180/np.pi} degrees')
            x1 = self.scaling_x*(pos_x+x_correction)+self.offset_left+self.offset # for left dot
            x2 = self.scaling_x*(pos_x-x_correction)+self.offset_left+self.offset # for right dot
            y1 = self.offset_top-self.scaling_y*(pos_z+z_correction)+self.offset
            y2 = self.offset_top-self.scaling_y*(pos_z-z_correction)+self.offset
            img = cv2.circle(img, (int(x1),
                             int(y1)), 5, self.renderColor, -1)
            img = cv2.circle(img, (int(x2),
                                   int(y2)), 5, self.renderColor, -1)
            img = cv2.line(img, (int(x1),
                                 int(y1)),
                           (int(x2),
                            int(y2)), self.renderColor, 2)
            return img

        def plot_y(img):
            """"Changes the plot so that you can see from the y axis side"""
            z_correction = self.arm_length * np.sin(roll)
            y_correction = self.arm_length * np.cos(roll)
            x1 = self.scaling_x*(pos_y+y_correction)+self.offset_left+self.offset # for left dot
            x2 = self.scaling_x*(pos_y-y_correction)+self.offset_left+self.offset # for right dot
            y1 = self.offset_top-self.scaling_y*(pos_z+z_correction)+self.offset
            y2 = self.offset_top-self.scaling_y*(pos_z-z_correction)+self.offset
            img = cv2.circle(img, (int(x1),
                                   int(y1)), 5, self.renderColor, -1)
            img = cv2.circle(img, (int(x2),
                                   int(y2)), 5, self.renderColor, -1)
            img = cv2.line(img, (int(x1),
                                 int(y1)),
                           (int(x2),
                           int(y2)), self.renderColor, 2)
            return img

        #checks which axis is selected to plot
        if self.axis_to_plot == 'x':
            img = plot_x(img)
        elif self.axis_to_plot == 'y':
            img = plot_y(img)

        # saving the pictures at the sampled timestep and length
        if self.save_render_image is True:
            if (t_n % self.timestep) < self.modulus_prev:
                if t_n <= self.sample_length:
                    if self.axis_to_plot == 'x':
                        self.final_image = plot_x(self.final_image)
                    elif self.axis_to_plot == 'y':
                        self.final_image = plot_y(self.final_image)
                else:
                    cv2.imwrite(f'../Crazyflie_Simulation/solid/Rendering/Images/final_image.png', self.final_image)

        self.modulus_prev = t_n % self.timestep

        data = img.tobytes("C")
        msg = Image(data=data, height=self.height, width=self.width, encoding="bgr8", step=3 * self.width)
        return dict(image=msg)


class HeightPID(eagerx.Node):
    @staticmethod
    @eagerx.register.spec("HeightPID", eagerx.Node)
    def spec(
            spec,
            name: str,
            rate: float,
    ):
        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = eagerx.process.ENVIRONMENT
        spec.config.inputs = ["current_height", "desired_height"]
        spec.config.outputs = ["new_action"]

        # Add space converters
        spec.inputs.current_height.space_converter = eagerx.SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[-1000, -1000, 0],
            high=[1000, 1000, 1000],
        )

        spec.inputs.desired_height.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                                [0],
                                                                                [65535], dtype="float32")

        spec.outputs.new_action.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                             [0],
                                                                             [65535], dtype="float32")

    def initialize(self):
        # Define values for kp, ki, kd
        self.kp = 0.2
        self.ki = 0.0001
        self.kd = 0.4
        self.pid = PID(kp=self.kp, ki=self.ki, kd=self.kd, rate=self.rate)
        self.gravity = 0.027 * 9.81

    @eagerx.register.states()
    def reset(self):
        self.pid.reset()

    # Force to PWM
    @staticmethod
    def force_to_pwm(force):
        # Just the inversion of pwm_to_force
        a = 4 * 2.130295e-11
        b = 4 * 1.032633e-6
        c = 5.485e-4 - force
        d = b ** 2 - 4 * a * c
        pwm = (-b + np.sqrt(d)) / (2 * a)
        return pwm

    @eagerx.register.inputs(current_height=Float32MultiArray, desired_height=Float32MultiArray)
    @eagerx.register.outputs(new_action=Float32MultiArray)
    def callback(self, t_n: float, current_height: Msg, desired_height: Msg):
        next_force = self.gravity + self.pid.next_action(current=current_height.msgs[-1].data[2],
                                                         desired=desired_height.msgs[-1].data[0])
        next_pwm = np.clip(self.force_to_pwm(next_force), 10000, 60000)
        return dict(new_action=Float32MultiArray(data=np.array([next_pwm])))


class ValidatePID(eagerx.Node):
    @staticmethod
    @eagerx.register.spec("ValidatePID", eagerx.Node)
    def spec(
            spec,
            name: str,
            rate: float,
    ):
        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = eagerx.process.ENVIRONMENT
        spec.config.inputs = ["current_position", "desired_position"]
        spec.config.outputs = ["new_attitude", "new_thrust"]

        # Add space converters
        spec.inputs.current_position.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                                  dtype="float32",
                                                                                  low=[-1000, -1000, 0],
                                                                                  high=[1000, 1000, 1000],
                                                                                  )

        spec.inputs.desired_position.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                                  dtype="float32",
                                                                                  low=[-1000, -1000, 0],
                                                                                  high=[1000, 1000, 1000],
                                                                                  )
        # for degrees output
        spec.outputs.new_attitude.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                               [-30, -30, -30],
                                                                               [30, 30, 30], dtype="float32")
        # for quaternion output
        # spec.outputs.new_attitude.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
        #                                                                        [-1, -1, -1, -1],
        #                                                                        [1, 1, 1, 1], dtype="float32")
        spec.outputs.new_thrust.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                             [0],
                                                                             [65535], dtype="float32")

    def initialize(self):
        # Define values for kp, ki, kd
        self.kp_x = 0.05    # 0.05
        self.ki_x = 0.0001
        self.kd_x = 0.06    # 0.06
        self.kp_z = 0.2
        self.ki_z = 0.0001
        self.kd_z = 0.4
        self.pid_x = PID(kp=self.kp_x, ki=self.ki_x, kd=self.kd_x, rate=self.rate)
        self.pid_z = PID(kp=self.kp_z, ki=self.ki_z, kd=self.kd_z, rate=self.rate)
        self.gravity = 0.027 * 9.81

    @eagerx.register.states()
    def reset(self):
        self.pid_x.reset()
        self.pid_z.reset()

    # Force to PWM
    @staticmethod
    def force_to_pwm(force):
        # Just the inversion of pwm_to_force
        a = 4 * 2.130295e-11
        b = 4 * 1.032633e-6
        c = 5.485e-4 - force
        d = b ** 2 - 4 * a * c
        pwm = (-b + np.sqrt(d)) / (2 * a)
        return pwm

    @eagerx.register.inputs(current_position=Float32MultiArray, desired_position=Float32MultiArray)
    @eagerx.register.outputs(new_attitude=Float32MultiArray, new_thrust=Float32MultiArray)
    def callback(self, t_n: float, current_position: Msg, desired_position: Msg):
        next_force_z = self.gravity + self.pid_z.next_action(current=current_position.msgs[-1].data[2],
                                                             desired=desired_position.msgs[-1].data[2])
        next_force_x = self.pid_x.next_action(current=current_position.msgs[-1].data[0],
                                              desired=desired_position.msgs[-1].data[0])
        next_pitch = np.clip(-np.arctan(next_force_x / next_force_z) * 180 / np.pi, -30, 30)
        next_thrust = np.cos(next_pitch * np.pi / 180) * next_force_z
        next_pwm = np.clip(self.force_to_pwm(next_thrust), 10000, 60000)

        if next_force_z <= 0:
            next_pwm = 14000

        # for quaternion
        # next_pitch = np.clip(-np.arctan(next_force_x / next_force_z), -np.pi/6, np.pi/6)
        # next_attitude = np.array(pybullet.getQuaternionFromEuler([0, next_pitch, 0]))

        # next_pitch = (current_position.msgs[-1].data[0] - 1) * 10
        # next_pitch = np.clip(next_pitch, -30, 30)

        # for degrees
        next_attitude = np.array([0, -next_pitch, 0])
        # next_attitude = np.array([0, 30, 0])
        print(50 * '=')
        print(f"next_attitude = {next_attitude}")
        print(f"next_pwm      = {next_pwm}")

        return dict(new_attitude=Float32MultiArray(data=next_attitude),
                    new_thrust=Float32MultiArray(data=np.array([next_pwm])))

# class AttitudePID(eagerx.Node):
#     @staticmethod
#     @eagerx.register.spec("AttitudePID", eagerx.Node)
#     def spec(
#             spec,
#             name: str,
#             rate: float,
#             n: int,
#     ):
#         # Performs all the steps to fill-in the params with registered info about all functions.
#         spec.initialize(AttitudePID)
#
#         # Modify default node params
#         spec.config.name = name
#         spec.config.rate = rate
#         spec.config.process = eagerx.process.ENVIRONMENT
#         spec.config.inputs = ["desired_attitude", "current_attitude"]
#         spec.config.outputs = ["new_attitude_rate"]
#
#         # Add space converters
#         spec.inputs.desired_attitude.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
#                                                                                   [-30, -30, -30],
#                                                                                   [30, 30, 30], dtype="float32")
#         spec.inputs.current_attitude.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
#                                                                                   [-1, -1, -1, -1],
#                                                                                   [1, 1, 1, 1], dtype="float32")
#         spec.outputs.new_attitude_rate.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
#                                                                                     [-90, -90, -90],
#                                                                                     [90, 90, 90], dtype="float32")
#
#     def initialize(self):
#         self.attitude_pid_yaw = PID(kp=6, ki=1, kd=0.35, rate=self.rate)
#         self.attitude_pid_pitch = PID(kp=6, ki=3, kd=0, rate=self.rate)
#         self.attitude_pid_roll = PID(kp=6, ki=3, kd=0, rate=self.rate)
#
#     @eagerx.register.states()
#     def reset(self):
#         self.attitude_pid_yaw.reset()
#         self.attitude_pid_pitch.reset()
#         self.attitude_pid_roll.reset()
#
#     @eagerx.register.inputs(desired_attitude=Float32MultiArray, current_attitude=Float32MultiArray)
#     @eagerx.register.outputs(new_attitude_rate=Float32MultiArray)
#     def callback(self, t_n: float, desired_attitude: Msg, current_attitude: Msg):
#         # Current attitude is a quaternion
#         current_attitude = current_attitude.msgs[-1].data
#         # Desired attitude is in degrees
#         desired_attitude = desired_attitude.msgs[-1].data
#         # Current attitude from quaternion to euler angles (RPY)
#         current_attitude_euler = pybullet.getEulerFromQuaternion(current_attitude)
#         # Current attitude from radians to degrees
#         current_attitude_deg = np.zeros(3)
#         for idx, ang in enumerate(current_attitude_euler):
#             current_attitude_deg[idx] = ang * 180 / np.pi
#
#         # print(f"Desired attitude: {desired_attitude[1]}")
#         # print(f"Current attitude: {current_attitude_deg[1]}")
#
#         next_roll = self.attitude_pid_roll.next_action(current=current_attitude_deg[0], desired=desired_attitude[0])
#         # todo: (-) minus sign (necessary to make it work)
#         next_pitch = self.attitude_pid_pitch.next_action(current=-current_attitude_deg[1], desired=desired_attitude[1])
#         next_yaw = self.attitude_pid_yaw.next_action(current=current_attitude_deg[2], desired=desired_attitude[2])
#         next_action = np.array([next_roll, next_pitch, next_yaw])
#         return dict(new_attitude_rate=Float32MultiArray(data=next_action))


# class AttitudeRatePID(eagerx.Node):
#     @staticmethod
#     @eagerx.register.spec("AttitudeRatePID", eagerx.Node)
#     def spec(
#             spec,
#             name: str,
#             rate: float,
#             n: int,
#     ):
#         # Performs all the steps to fill-in the params with registered info about all functions.
#         spec.initialize(AttitudeRatePID)
#
#         # Modify default node params
#         spec.config.name = name
#         spec.config.rate = rate
#         spec.config.process = eagerx.process.ENVIRONMENT
#         spec.config.inputs = ["desired_rate", "current_rate"]
#         spec.config.outputs = ["new_motor_control"]
#
#         # Add space converters
#         spec.inputs.desired_rate.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
#                                                                               [-90, -90, -90],
#                                                                               [90, 90, 90], dtype="float32")
#         spec.inputs.current_rate.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
#                                                                               [50000, 50000, 50000],
#                                                                               [50000, 50000, 50000], dtype="float32")
#         spec.outputs.new_motor_control.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
#                                                                                     [-32767, -32767, -32767],
#                                                                                     [32767, 32767, 32767],
#                                                                                     dtype="float32")
#
#     def initialize(self):
#         self.attitude_rate_pid_yaw = PID(kp=120, ki=16.7, kd=0, rate=self.rate)
#         self.attitude_rate_pid_pitch = PID(kp=250, ki=500, kd=2.5, rate=self.rate)
#         self.attitude_rate_pid_roll = PID(kp=250, ki=500, kd=2.5, rate=self.rate)
#
#     @eagerx.register.states()
#     def reset(self):
#         self.attitude_rate_pid_yaw.reset()
#         self.attitude_rate_pid_pitch.reset()
#         self.attitude_rate_pid_roll.reset()
#
#     @eagerx.register.inputs(desired_rate=Float32MultiArray, current_rate=Float32MultiArray)
#     @eagerx.register.outputs(new_motor_control=Float32MultiArray)
#     def callback(self, t_n: float, desired_rate: Msg, current_rate: Msg):
#         current_attitude_rate = current_rate.msgs[-1].data  # (vx, vy, vz) Roll, pitch, yaw
#         desired_attitude_rate = desired_rate.msgs[-1].data
#         # print(f"Current rate: {current_attitude_rate[1]}")
#         # print(f"Desired rate: {desired_attitude_rate[1]}")
#         # print("-" * 50)
#         next_roll_rate = self.attitude_rate_pid_roll.next_action(current=current_attitude_rate[0],
#                                                                  desired=desired_attitude_rate[0])
#         next_pitch_rate = self.attitude_rate_pid_pitch.next_action(current=current_attitude_rate[1],
#                                                                    desired=desired_attitude_rate[1])
#         next_yaw_rate = self.attitude_rate_pid_yaw.next_action(current=current_attitude_rate[2],
#                                                                desired=desired_attitude_rate[2])
#
#         next_action = np.array([next_roll_rate, next_pitch_rate, next_yaw_rate])
#         # print(next_action)
#         return dict(new_motor_control=Float32MultiArray(data=next_action))


# class PowerDistribution(eagerx.Node):
#     @staticmethod
#     @eagerx.register.spec("PowerDistribution", eagerx.Node)
#     def spec(
#             spec,
#             name: str,
#             rate: float,
#             n: int,
#     ):
#         # Performs all the steps to fill-in the params with registered info about all functions.
#         spec.initialize(PowerDistribution)
#
#         # Modify default node params
#         spec.config.name = name
#         spec.config.rate = rate
#         spec.config.process = eagerx.process.ENVIRONMENT
#         spec.config.inputs = ["desired_thrust", "calculated_control"]
#         spec.config.outputs = ["pwm_signal"]
#
#         # Add space converters
#         spec.inputs.desired_thrust.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
#                                                                                 [0],
#                                                                                 [65535], dtype="float32")
#         spec.inputs.calculated_control.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
#                                                                                     [-32767, -32767, -32767],
#                                                                                     [32767, 32767, 32767],
#                                                                                     dtype="float32")
#         spec.outputs.pwm_signal.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
#                                                                              [0, 0, 0, 0],
#                                                                              [65535, 65535, 65535, 65535],
#                                                                              dtype="float32")
#
#     def initialize(self):
#         pass
#
#     @eagerx.register.states()
#     def reset(self):
#         pass
#
#     # limit motorpowers definition
#     def limitThrust(self, value):
#         if value > 65535:
#             value = 65535
#         elif value < 0:
#             value = 0
#         return value
#
#     # limit minimum idle Thrust definition
#     def limitIdleThrust(self, value, minimum):
#         if (value < minimum):
#             value = minimum
#         return value
#
#     @eagerx.register.inputs(desired_thrust=Float32MultiArray, calculated_control=Float32MultiArray)
#     @eagerx.register.outputs(pwm_signal=Float32MultiArray)
#     def callback(self, t_n: float, desired_thrust: Msg, calculated_control: Msg):
#         desired_thrust.msgs[-1].data  # set input at 30000, moet nog vanuit env/actions komen
#
#         # define variables from inputs
#         calculated_control_input = calculated_control.msgs[-1].data  # roll pitch yaw
#         roll = calculated_control.msgs[-1].data[0]
#         pitch = calculated_control.msgs[-1].data[1]
#         yaw = calculated_control.msgs[-1].data[2]
#         desired_thrust_input = desired_thrust.msgs[-1].data[0]
#         # print(f"======\n calculated control = {calculated_control_input} \n desired thrust {desired_thrust_input}") #debug
#
#         # limit motorpowers
#         motorPower_m1 = self.limitThrust(desired_thrust_input - roll + pitch + yaw)
#         motorPower_m2 = self.limitThrust(desired_thrust_input - roll - pitch - yaw)
#         motorPower_m3 = self.limitThrust(desired_thrust_input + roll - pitch + yaw)
#         motorPower_m4 = self.limitThrust(desired_thrust_input + roll + pitch - yaw)
#
#         # limit minimum idle Thrust
#         minimumIdleThrust = 0  # PWM signal, default = 0
#         motorPower_m1 = self.limitIdleThrust(motorPower_m1, minimumIdleThrust)
#         motorPower_m2 = self.limitIdleThrust(motorPower_m2, minimumIdleThrust)
#         motorPower_m3 = self.limitIdleThrust(motorPower_m3, minimumIdleThrust)
#         motorPower_m4 = self.limitIdleThrust(motorPower_m4, minimumIdleThrust)
#
#         # print(motorPower_m4) # debug
#
#         new_pwm_signal = np.array(
#             [motorPower_m1, motorPower_m2, motorPower_m3, motorPower_m4])  # enginenode verwacht force
#         # new_pwm_signal = np.array([3,3,0])
#         return dict(pwm_signal=Float32MultiArray(data=new_pwm_signal))

# class StateEstimator(eagerx.Node):
#     @staticmethod
#     @eagerx.register.spec("StateEstimator", eagerx.Node)
#     def spec(
#             spec,
#             name: str,
#             rate: float,
#             n: int,
#     ):
#         # Performs all the steps to fill-in the params with registered info about all functions.
#         spec.initialize(StateEstimator)
#
#         # Modify default node params
#         spec.config.name = name
#         spec.config.rate = rate
#         spec.config.process = eagerx.process.ENVIRONMENT
#         spec.config.inputs = ["angular_velocity", "acceleration", "orientation"]
#         spec.config.outputs = ["orientation"]
#
#         # Add space converters
#         spec.inputs.angular_velocity.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
#                                                                                   [0, 0, 0],
#                                                                                   [3, 3, 3], dtype="float32")
#         spec.inputs.acceleration.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
#                                                                               [0, 0, 0],
#                                                                               [3, 3, 3], dtype="float32")
#         spec.inputs.orientation.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
#                                                                              [0, 0, 0],
#                                                                              [3, 3, 3], dtype="float32")
#         spec.outputs.orientation.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
#                                                                               [0, 0, 0],
#                                                                               [3, 3, 3], dtype="float32")
#
#     def initialize(self):
#         pass
#
#     @eagerx.register.states()
#     def reset(self):
#         pass
#
#     @eagerx.register.inputs(angular_velocity=Float32MultiArray, acceleration=Float32MultiArray,
#                             orientation=Float32MultiArray)
#     @eagerx.register.outputs(orientation=Float32MultiArray)
#     def callback(self, t_n: float, angular_velocity: Msg, acceleration: Msg, orientation: Msg):
#         attitude = orientation.msgs[-1].data
#         return dict(orientation=Float32MultiArray(data=attitude))
