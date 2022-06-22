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



class MakePicture(eagerx.Node):
    """Class that is used to make a (square) picture of the crazyflie postion in the world"""
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
            engine_mode: str,
    ):
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(MakePicture)

        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = eagerx.process.ENVIRONMENT
        spec.config.inputs = ["position", "orientation"]
        spec.config.outputs = ["image", "time"]

        spec.config.save_render_image = save_render_image if save_render_image else False
        spec.config.saveToPreviousRender = saveToPreviousRender if saveToPreviousRender else False
        spec.config.renderColor = renderColor if renderColor else "black"
        spec.config.axisToPlot = axisToPlot if axisToPlot else "x"
        spec.config.max_steps = max_steps if max_steps else 500
        spec.config.engine_mode = engine_mode if engine_mode else "Pybullet"

        # Add space converters
        spec.inputs.position.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                          [0, 0, 0],
                                                                          [3, 3, 3], dtype="float32")
        spec.inputs.orientation.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                             [0, 0, 0],
                                                                             [3, 3, 3], dtype="float32")
        spec.outputs.time.space_converter = eagerx.SpaceConverter.make("Space_Float32",
                                                                       0,
                                                                       9999, dtype="float32")

    def initialize(self, save_render_image, saveToPreviousRender, renderColor, axisToPlot, max_steps, engine_mode):
        # Change render settings here
        self.height = 940  # set render height [px]
        self.width = 940  # set render width [px]
        self.offset = 70  # offset of the picture from the sides [px]
        self.timestep = 0.1  # set timestep for render [s]
        self.length = 10
        self.text_height = 4
        self.font = cv2.FONT_HERSHEY_PLAIN
        # eight: xrange = [-2.5, 2.5] yrange = [0.9, 3.2]
        # line: xrange = [-3, 3] yrange = [0.9, 2.1]
        # rectangle: xrange = [-1.5, 1.5] yrange = [0.8, 3.2]
        # triangle: xrange = [-1.5, 1.5] yrange = [0.8, 3]
        self.xrange = [-3, 3]  # range of x (x in 2D image = to right) [m]
        self.yrange = [0.9, 3.2]  # range of y (y in 2D image = up) [m]
        self.amountOfDivisions = 11  # amount of divisions on axis, def=9

        # set drone arm lengths
        self.arm_length = 0.028 * 4

        # set engine mode
        self.engine_mode = engine_mode

        # Settings which are set when creating the MakePicture Node
        self.save_render_image = save_render_image  # enable or disable render from 2D plot
        self.saveToPreviousRender = saveToPreviousRender  # saves render on top of last render
        self.renderColor = renderColor  # blue, red or black for now
        self.axis_to_plot = axisToPlot  # 'x' or 'y' right now

        self.sample_length = (1 * max_steps) / (self.rate)
        self.checkIfSaved = False

        self.i = 0
        self.max_steps = max_steps
        self.inverColorRange = False

        # AUTO INITIALIZATIONS
        print("Current engine:", self.engine_mode)

        # init initial image
        self.modulus_prev = 1000
        if self.saveToPreviousRender == True:
            self.final_image = cv2.imread(f'../Crazyflie_Simulation/solid/Rendering/final_image.png')
        else:
            self.final_image = 0

        # init render color gradients in BGR format
        if self.renderColor == "greenblue":
            self.renderColorTest = [np.linspace(155, 255, max_steps), np.linspace(255, 155, max_steps),
                                    np.zeros(max_steps)]
        elif self.renderColor == "purplepink":
            self.renderColorTest = [np.linspace(155, 255, max_steps), np.zeros(max_steps),
                                    np.linspace(155, 255, max_steps)]
        elif self.renderColor == "greyblack":
            self.renderColorTest = [np.linspace(200, 50, max_steps), np.linspace(200, 50, max_steps),
                                    np.linspace(200, 50, max_steps)]
        else:
            self.renderColor = [255, 0, 0]

        # init axis for render
        self.y_axis = np.linspace(self.yrange[0], self.yrange[1], self.amountOfDivisions)
        for idx, y in enumerate(self.y_axis):
            self.y_axis[idx] = '%.2f' % (y)
        self.x_axis = np.linspace(self.xrange[1], self.xrange[0], self.amountOfDivisions)
        for idx, x in enumerate(self.x_axis):
            self.x_axis[idx] = '%.2f' % (x)

        # calculate scaling and offsets
        self.scaling_x = (self.width - 2 * self.offset) / (
                max(self.x_axis) - min(self.x_axis))  # set scaling per division
        self.scaling_y = (self.height - 2 * self.offset) / (max(self.y_axis) - min(self.y_axis))
        self.offset_left = abs(
            min(self.x_axis) / (max(self.x_axis) - min(self.x_axis)) * (self.width - 2 * self.offset))  # set offset
        self.offset_top = abs(
            max(self.y_axis) / (max(self.y_axis) - min(self.y_axis)) * (self.height - 2 * self.offset))

        self.scaling_drone = 200
    @eagerx.register.states()
    def reset(self):
        pass

    @eagerx.register.inputs(position=Float32MultiArray, orientation=Float32MultiArray)
    @eagerx.register.outputs(image=Image, time=Float32)
    def callback(self, t_n: float, position: Msg, orientation: Msg):
        """"Makes the picture in the seleted axis for every timestep

        :param position: the position of the Crazyflie in x,y,z in the world frame
        :param orientation: the orientation of the Crazyflie in quaternions in the world frame
        :return image: return the picture with the axis of the Crazyflie
        :type image: Image type
        """
        pos_x, pos_y, pos_z = position.msgs[-1].data[0], position.msgs[-1].data[1], position.msgs[-1].data[2]

        if len(orientation.msgs[-1].data) == 4:
            euler_orientation = pybullet.getEulerFromQuaternion(orientation.msgs[-1].data)
        else:
            euler_orientation = np.array(orientation.msgs[-1].data) * np.pi / 180
        roll, pitch, yaw = euler_orientation[0], -euler_orientation[1], euler_orientation[2]

        # HARDCODED CORRECTION FOR Ode
        if self.engine_mode == "Ode":
            roll, pitch, yaw = roll * -180 / np.pi, pitch * -180 / np.pi, yaw * -180 / np.pi

        img = np.zeros((self.height, self.width, 3), np.uint8)
        img[:, :] = (255, 255, 255)

        for i in range(self.amountOfDivisions):  # add coordinate system to the rendered picture y axis
            y_axis = self.y_axis
            x = self.offset
            y = int(self.height - i * ((self.height - 2 * self.offset) / (self.amountOfDivisions - 1)) - self.offset)
            img = cv2.line(img, (x, y), (x - self.length, y), (0, 0, 0), 1)  # make markers on y-axis
            img = cv2.putText(img, str(y_axis[i]), (5, y + self.text_height), self.font, 1.3, (0, 0, 0), 2)

        for i in range(self.amountOfDivisions):  # add coordinate system to the rendered picture x axis
            x_axis = self.x_axis
            x = int(self.width - i * ((self.width - 2 * self.offset) / (self.amountOfDivisions - 1)) - self.offset)
            y = self.height - self.offset
            img = cv2.line(img, (x, self.height - self.offset), (x, self.height - self.offset + self.length), (0, 0, 0),
                           1)  # make markers on x-axis
            img = cv2.putText(img, str(x_axis[i]), (x - self.text_height * 4, y + 25), self.font, 1.3, (0, 0, 0), 2)

        # render axis labels
        img = cv2.putText(img, str("Z-axis [m]"), (5, self.offset - 5 * self.text_height), self.font, 1.3, (0, 0, 0), 2)
        if self.axis_to_plot == 'x':
            img = cv2.putText(img, str("X-axis [m]"),
                              (self.width - 2 * self.offset - self.text_height * 0, self.height - 20), self.font, 1.3,
                              (0, 0, 0), 2)
        elif self.axis_to_plot == 'y':
            img = cv2.putText(img, str("Y-axis [m]"),
                              (self.width - 2 * self.offset - self.text_height * 0, self.height - 20), self.font, 1.3,
                              (0, 0, 0), 2)

        #  create border
        img = cv2.rectangle(img, (self.offset, self.offset), (self.height - self.offset, self.width - self.offset),
                            (0, 0, 0), 1)

        # give the sampled image an axis like for the render image
        if type(self.final_image) is int:
            self.final_image = img

        if self.inverColorRange == False:
            i = self.i
        else:
            i = self.max_steps - 1 - self.i

        def plot_x(img):
            """"Changes the plot so that you can see from the x axis side"""
            z_correction = self.arm_length * np.sin(-pitch)
            x_correction = self.arm_length * np.cos(-pitch)
            x1 = self.scaling_x * pos_x + self.scaling_drone * x_correction + self.offset_left + self.offset  # for left dot
            x2 = self.scaling_x * pos_x - self.scaling_drone * x_correction + self.offset_left + self.offset  # for right dot
            y1 = self.offset_top - self.scaling_y * pos_z - self.scaling_drone * z_correction + self.offset
            y2 = self.offset_top - self.scaling_y * pos_z + self.scaling_drone * z_correction + self.offset
            img = cv2.circle(img, (int(x1),
                                   int(y1)), 5,
                             [self.renderColorTest[0][i], self.renderColorTest[1][i], self.renderColorTest[2][i]], -1)
            img = cv2.circle(img, (int(x2),
                                   int(y2)), 5,
                             [self.renderColorTest[0][i], self.renderColorTest[1][i], self.renderColorTest[2][i]], -1)
            img = cv2.line(img, (int(x1),
                                 int(y1)),
                           (int(x2),
                            int(y2)),
                           [self.renderColorTest[0][i], self.renderColorTest[1][i], self.renderColorTest[2][i]], 2)
            return img

        def plot_y(img):
            """"Changes the plot so that you can see from the y axis side"""
            z_correction = self.arm_length * np.sin(roll)
            y_correction = self.arm_length * np.cos(roll)
            x1 = self.scaling_x * pos_y + self.scaling_drone * y_correction + self.offset_left + self.offset  # for left dot
            x2 = self.scaling_x * pos_y - self.scaling_drone * y_correction + self.offset_left + self.offset  # for right dot
            y1 = self.offset_top - self.scaling_y * pos_z + self.scaling_drone * z_correction + self.offset
            y2 = self.offset_top - self.scaling_y * pos_z - self.scaling_drone * z_correction + self.offset
            img = cv2.circle(img, (int(x1),
                                   int(y1)), 5,
                             [self.renderColorTest[0][i], self.renderColorTest[1][i], self.renderColorTest[2][i]], -1)
            img = cv2.circle(img, (int(x2),
                                   int(y2)), 5,
                             [self.renderColorTest[0][i], self.renderColorTest[1][i], self.renderColorTest[2][i]], -1)
            img = cv2.line(img, (int(x1),
                                 int(y1)),
                           (int(x2),
                            int(y2)),
                           [self.renderColorTest[0][i], self.renderColorTest[1][i], self.renderColorTest[2][i]], 2)
            return img

        # checks which axis is selected to plot
        if self.axis_to_plot == 'x':
            img = plot_x(img)
        elif self.axis_to_plot == 'y':
            img = plot_y(img)

        # saving the pictures at the sampled timestep and length
        if self.save_render_image is True:
            if (t_n % self.timestep) < self.modulus_prev and t_n <= self.sample_length:
                # if t_n <= self.sample_length:
                if self.axis_to_plot == 'x':
                    self.final_image = plot_x(self.final_image)
                elif self.axis_to_plot == 'y':
                    self.final_image = plot_y(self.final_image)

            elif t_n > self.sample_length and self.checkIfSaved == False:
                print("Saving image...")
                self.final_image = self.legenda(self.final_image)
                if self.axis_to_plot == 'x':
                    self.final_image = plot_x(self.final_image)
                elif self.axis_to_plot == 'y':
                    self.final_image = plot_y(self.final_image)

                if self.saveToPreviousRender == True:
                    cv2.imwrite(f'../Crazyflie_Simulation/solid/Rendering/final_image.png', self.final_image)
                elif self.checkIfSaved == False:
                    cv2.imwrite(f'../Crazyflie_Simulation/solid/Rendering/final_image.png', self.final_image)

                self.checkIfSaved = True

        self.modulus_prev = t_n % self.timestep

        data = img.tobytes("C")
        msg = Image(data=data, height=self.height, width=self.width, encoding="bgr8", step=3 * self.width)


        # set current timestep
        if self.i < (self.max_steps - 1):
            self.i += 1

        return dict(image=msg, time=Float32(data=t_n))

    @staticmethod
    def legenda(im):
        green = (0, 255, 0)
        rectangle_start = [940 - 250 - 70, 70]
        rectangle_size = [250, 90]

        # rectangle_pos = [(630, 750), (820, 820)]
        rectangle_pos = [(rectangle_start[0], rectangle_start[1]),
                         (rectangle_start[0] + rectangle_size[0], rectangle_start[1] + rectangle_size[1])]
        length_line = 40
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 0.8

        im = cv2.rectangle(im, rectangle_pos[0], rectangle_pos[1], (0, 0, 0), 1)
        red = [np.linspace(255, 155, length_line), np.zeros(length_line), np.linspace(155, 255, length_line)]
        black = [np.linspace(200, 50, length_line), np.linspace(200, 50, length_line),
                 np.linspace(200, 50, length_line)]
        for i in range(length_line):
            im = cv2.line(im, (int(rectangle_pos[0][0] + 10 + i * 1.5), rectangle_pos[0][1] + 60),
                          (rectangle_pos[0][0] + 10 + i, rectangle_pos[0][1] + 60), (red[0][i], red[1][i], red[2][i]),
                          10)
            im = cv2.line(im, (int(rectangle_pos[0][0] + 10 + i * 1.5), rectangle_pos[0][1] + 30),
                          (rectangle_pos[0][0] + 10 + i, rectangle_pos[0][1] + 30),
                          (black[0][i], black[1][i], black[2][i]), 10)
            # im = cv2.line(im, (int(rectangle_pos[0][0] + 10 + i * 1.5), rectangle_pos[0][1] + 90),
            #               (rectangle_pos[0][0] + 10 + i, rectangle_pos[0][1] + 90), green, 10)
        # im = cv2.putText(im, "1-4", (rectangle_pos[0][0] + 10, rectangle_pos[0][1] + 125), font, fontScale,
        #                  green, 2)
        im = cv2.putText(im, "= white-box", (rectangle_pos[0][0] + 80, rectangle_pos[0][1] + 35), font, fontScale,
                         (0, 0, 0), 2)
        im = cv2.putText(im, "= black-box", (rectangle_pos[0][0] + 80, rectangle_pos[0][1] + 65), font, fontScale,
                         (0, 0, 0), 2)
        # im = cv2.putText(im, "= trajectory", (rectangle_pos[0][0] + 80, rectangle_pos[0][1] + 95), font, fontScale,
        #                  (0, 0, 0), 2)
        # im = cv2.putText(im, "= points", (rectangle_pos[0][0] + 80, rectangle_pos[0][1] + 125), font, fontScale,
        #                  (0, 0, 0), 2)

        return im


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
        """ In the callback the next force is calculated. After that this is converted to PWM.
        :param current_height: the current height of the drone.
        :param desired_height: the desired height of the drone.

        :return dictionary containing the new PWM value.
         """
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
            engine_mode: str,
    ):
        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.engine_mode = engine_mode
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

    def initialize(self, engine_mode):
        # Define values for kp, ki, kd
        self.kp_x = 0.05  # 0.05
        self.ki_x = 0.0001
        self.kd_x = 0.06  # 0.06
        self.kp_z = 0.2
        self.ki_z = 0.0001
        self.kd_z = 0.4
        self.pid_x = PID(kp=self.kp_x, ki=self.ki_x, kd=self.kd_x, rate=self.rate)
        self.pid_z = PID(kp=self.kp_z, ki=self.ki_z, kd=self.kd_z, rate=self.rate)
        self.gravity = 0.027 * 9.81
        self.engine_mode = engine_mode
        self.i = 0

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
        """ In the callback the desired attitude and thrust are calculated with a PID.
        :param current_position: the current position of the drone ([x, y, z]).
        :param desired_position: the desired position which could be used as an input.
                                 We used one of the trajectories as a setpoint.

        :return dictionary containing the new attitude and thrust.
                These are the desired attitude and thrust for the drone because this node is used in the agnostic part.
        """
        current_pos = current_position.msgs[-1].data
        # setpoint = desired_position.msgs[-1].data
        # Choose your validate function

        # Choose trajectory
        # For eight_trajectory: 3000 steps, startpoint: 0, 0, 2
        # For line_trajectory: 1750 steps, startpoint: 0, 0, 2
        # For rectangle_trajectory: 3750 steps, startpoint: -1, 0, 1. Optional: self.timestep to 0.15
        # For triangle_trajectory: 1500 steps, startpoint: -1, 0, 1
        setpoint = self.line_trajectory()

        # print(setpoint)
        next_force_z = self.gravity + self.pid_z.next_action(current=current_pos[2],
                                                             desired=setpoint[2])
        next_force_x = self.pid_x.next_action(current=current_pos[0],
                                              desired=setpoint[0])
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
        # next_pitch = 30
        if self.engine_mode == 'Ode':
            next_attitude = np.array([0, -next_pitch, 0])
        elif self.engine_mode == 'Pybullet':
            next_attitude = np.array([0, next_pitch, 0])
        else:
            print('Wrong Enginemode selected')
        # next_attitude = np.array([0, 30, 0])
        # print(f"next_attitude = {next_attitude}")
        # print(f"next_pwm      = {next_pwm}")

        return dict(new_attitude=Float32MultiArray(data=next_attitude),
                    new_thrust=Float32MultiArray(data=np.array([next_pwm])))

    def line_trajectory(self, length=6, center=[0, 0, 1], speed=5):
        """Function that sets the setpoint to form a line.
        It sends one end of the line first and after a certain
        time the other endpoint.

        :param: length: horizontal length of the line
        :param: center: center point of the line
        :param: speed: determines the speed at which the quadcopter flies the trajectory
        :return: setpoint: returns setpoint to send to the trajectory PID
        """
        time = 2 * length / speed
        steps = int(time * self.rate)
        center = np.array(center)
        point_l = np.array([-length / 2, 0, 0]) + center
        point_r = np.array([length / 2, 0, 0]) + center
        points = [point_l, point_r]

        i = self.i % steps
        if self.i / steps % 2 <= 1:
            setpoint = points[0]
        else:
            setpoint = points[1]

        self.i += 1
        return setpoint

    def rectangle_trajectory(self, length=2, start=[0, 0, 2], speed=1):
        """Function that sets the setpoint to form a rectangle.
        It sends one end of the rectangle first and after a certain
        time the other corner and this repeats for all corners.

        :param: length: length of the sides of the triangle
        :param: start: center of the rectangle
        :param: speed: determines the speed at which the quadcopter flies the trajectory
        :return: setpoint: returns setpoint to send to the trajectory PID
        """
        time = 8 * length / speed
        start = np.array(start) + np.array([-length / 2, 0, -length / 2])
        steps = int(time * self.rate)
        point_zero = start
        point_one = start + np.array([length, 0, 0])
        point_two = start + np.array([length, 0, length])
        point_three = start + np.array([0, 0, length])
        points = [point_zero, point_one, point_two, point_three]
        # print(f'points is {points}')

        i = self.i % steps

        if i < steps / 4:
            setpoint = points[1]
        elif i < 2 * steps / 4:
            setpoint = points[2]
        elif i < 3 * steps / 4:
            setpoint = points[3]
        else:
            setpoint = points[0]

        self.i += 1
        return setpoint

    def circular_trajectory(self, radius=1, origin=[0, 0, 2], speed=1):
        """Function that sets the setpoint to form a circle.
        It sends a point of the circle and after a certain
        time another further point on the circle and this repeats
        for the whole circle.

        :param: radius: radius of the circle
        :param: origin: center point of the circle
        :param: speed: determines the speed at which the quadcopter flies the trajectory
        :return: setpoint: returns setpoint to send to the trajectory PID
        """
        circle_length = radius * 2 * np.pi
        time = circle_length / speed
        steps = int(time * self.rate)
        theta = np.linspace(-np.pi / 2, 3 / 2 * np.pi, steps)

        i = self.i % steps
        x = np.cos(theta[i]) * radius
        y = 0
        z = np.sin(theta[i]) * radius
        setpoint = np.array([x + origin[0], y, z + origin[2]])

        self.i += 1

        return setpoint

    def triangle_trajectory(self, line_length=2, startpoint=[-1, 0, 1], speed=1):
        """Function that sets the setpoint to form a triangle.
        It sends a point of the triangle and after a certain
        time another further point on the triangle and this repeats
        for the whole triangle.

        :param: line_length: length of the sides of the triangle
        :param: startpoint: point used to start the triangle
        :param: speed: determines the speed at which the quadcopter flies the trajectory
        :return: setpoint: returns setpoint to send to the trajectory PID
        """
        time = line_length * 3 / speed
        steps = int(time * self.rate)
        startpoint = np.array(startpoint)
        point1 = startpoint
        point2 = point1 + np.array([line_length, 0, 0])
        point3 = point2 + np.array([-np.cos(np.pi / 3) * line_length, 0, np.sin(np.pi / 3) * line_length])
        points = np.array([point2, point3, point1])

        i = self.i % steps
        if i < steps / 3:
            segment = points[0] - points[2]
            segment_part = segment / (steps / 3)
            setpoint = points[2] + segment_part * i
        elif i < 2 * steps / 3:
            segment = points[1] - points[0]
            segment_part = segment / (steps / 3)
            setpoint = points[0] + segment_part * (i - steps / 3)
        else:
            segment = points[2] - points[1]
            segment_part = segment / (steps / 3)
            setpoint = points[1] + segment_part * (i - 2 * steps / 3)

        self.i += 1
        return setpoint

    def eight_trajectory(self, radius=1, origin=[0, 0, 2], speed=1):
        """Function that sets the setpoint to form a lemniscate.
        It sends a point of the lemniscate and after a certain
        time another further point on the lemniscate and this repeats
        for the whole lemniscate.

        :param: radius: radius of the circle
        :param: origin: start point of the lemniscate
        :param: speed: determines the speed at which the quadcopter flies the trajectory
        :return: setpoint: returns setpoint to send to the trajectory PID
        """
        origin = np.array(origin)
        origin_r = np.array([radius, 0, 0]) + origin
        origin_l = np.array([-radius, 0, 0]) + origin
        circle_length = radius * 2 * np.pi
        time = circle_length / speed
        steps = int(time * self.rate)
        theta = np.linspace(-np.pi, np.pi, steps)

        i = self.i % steps
        if self.i / steps % 2 <= 1:
            x = np.cos(theta[i]) * radius
            y = 0
            z = np.sin(theta[i]) * radius
            setpoint = np.array([x + origin_r[0], y, z + origin_r[2]])
        else:
            x = np.cos(theta[i]) * radius
            y = 0
            z = np.sin(theta[i]) * radius
            setpoint = np.array([- x + origin_l[0], y, z + origin_l[2]])

        self.i += 1

        return setpoint
