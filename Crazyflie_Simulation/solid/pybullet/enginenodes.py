from typing import Optional, List
from scipy.spatial.transform import Rotation
import pybullet
import eagerx
from Crazyflie_Simulation.solid.pid import PID
import cv2
# IMPORT ROS
from std_msgs.msg import UInt64, Float32MultiArray
from sensor_msgs.msg import Image

# IMPORT EAGERX
from eagerx.core.specs import NodeSpec
from eagerx.core.constants import process as p
from eagerx.utils.utils import Msg
from eagerx.core.entities import EngineNode, SpaceConverter
import eagerx.core.register as register
import numpy as np


class AttitudePID(EngineNode):
    @staticmethod
    @eagerx.register.spec("AttitudePID", EngineNode)
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
                                                                                  [-30, -30, -30],
                                                                                  [30, 30, 30], dtype="float32")
        spec.outputs.new_attitude_rate.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                                    [-90, -90, -90],
                                                                                    [90, 90, 90], dtype="float32")

    def initialize(self):
        # Constants calculated from Crazyflie: kp = kp, ki = ki * new_rate / old_rate, kd = kd * old_rate / new_rate
        # Make the pid-controller objects
        self.attitude_pid_yaw = PID(kp=6, ki=0.2, kd=1.75, rate=self.rate)
        self.attitude_pid_pitch = PID(kp=6, ki=0.6, kd=0, rate=self.rate)
        self.attitude_pid_roll = PID(kp=6, ki=0.6, kd=0, rate=self.rate)

    @eagerx.register.states()
    def reset(self):
        self.attitude_pid_yaw.reset()
        self.attitude_pid_pitch.reset()
        self.attitude_pid_roll.reset()

    @eagerx.register.inputs(desired_attitude=Float32MultiArray, current_attitude=Float32MultiArray)
    @eagerx.register.outputs(new_attitude_rate=Float32MultiArray)
    def callback(self, t_n: float, desired_attitude: Msg, current_attitude: Msg):
        """ This node implements the attitude PID controller from the Crazyflie.
        There is a PID controller for every angle (yaw, pitch and roll) calculating the desired attitude rate.

        :param desired_attitude: The desired orientation in euler angles in degrees
        :param current_attitude: The current orientation in euler angles in degrees

        :return new_attitude_rate: The desired attitude rate (angular velocity) in euler angles in degrees per second send to the AttitudeRatePID node
        """
        # Current attitude in degrees
        current_attitude = current_attitude.msgs[-1].data
        # Desired attitude in degrees
        desired_attitude = desired_attitude.msgs[-1].data

        next_roll = self.attitude_pid_roll.next_action(current=current_attitude[0], desired=desired_attitude[0])
        next_pitch = self.attitude_pid_pitch.next_action(current=current_attitude[1], desired=desired_attitude[1])
        next_yaw = self.attitude_pid_yaw.next_action(current=current_attitude[2], desired=desired_attitude[2])
        next_action = np.array([next_roll, next_pitch, next_yaw])
        return dict(new_attitude_rate=Float32MultiArray(data=next_action))


class AttitudeRatePID(EngineNode):
    @staticmethod
    @eagerx.register.spec("AttitudeRatePID", EngineNode)
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
        # Constants calculated from Crazyflie: kp = kp, ki = ki * new_rate / old_rate, kd = kd * old_rate / new_rate
        self.attitude_rate_pid_yaw = PID(kp=120, ki=3.34, kd=0, rate=self.rate)
        self.attitude_rate_pid_pitch = PID(kp=250, ki=100, kd=12.5, rate=self.rate)  # 250, 500, 2.5
        self.attitude_rate_pid_roll = PID(kp=250, ki=100, kd=12.5, rate=self.rate)

    @eagerx.register.states()
    def reset(self):
        self.attitude_rate_pid_yaw.reset()
        self.attitude_rate_pid_pitch.reset()
        self.attitude_rate_pid_roll.reset()

    @eagerx.register.inputs(desired_rate=Float32MultiArray, current_rate=Float32MultiArray)
    @eagerx.register.outputs(new_motor_control=Float32MultiArray)
    def callback(self, t_n: float, desired_rate: Msg, current_rate: Msg):
        """ This node implements the attitude rate PID controller from the Crazyflie.
        There is a PID controller for every angle (yaw, pitch and roll) calculating the PWM signal differences
        to cause the rotation (roll, pitch and yaw).

        :param desired_rate: The desired attitude rate of the Crazyflie (roll, pitch, yaw) in degrees per second
        :param current_rate: The current attitude rate of the Crazyflie (roll, pitch, yaw) in degrees per second

        :return new_motor_control: The PWM signal difference per angle (roll, pitch, yaw) in a range [-32767, 32767] send to the PowerDistribution node
        """
        current_attitude_rate = current_rate.msgs[-1].data  # (vx, vy, vz) Roll, pitch, yaw
        desired_attitude_rate = desired_rate.msgs[-1].data

        next_roll_rate = self.attitude_rate_pid_roll.next_action(current=current_attitude_rate[0],
                                                                 desired=desired_attitude_rate[0])
        next_pitch_rate = self.attitude_rate_pid_pitch.next_action(current=current_attitude_rate[1],
                                                                   desired=desired_attitude_rate[1])
        next_yaw_rate = self.attitude_rate_pid_yaw.next_action(current=current_attitude_rate[2],
                                                               desired=desired_attitude_rate[2])

        next_action = np.array([next_roll_rate, next_pitch_rate, next_yaw_rate])
        return dict(new_motor_control=Float32MultiArray(data=next_action))


class PowerDistribution(EngineNode):
    @staticmethod
    @eagerx.register.spec("PowerDistribution", EngineNode)
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

    @staticmethod
    def limitThrust(value):
        """ Function to limit the maximum PWM value """
        if value > 65535:
            value = 65535
        elif value < 0:
            value = 0
        return value

    @staticmethod
    def limitIdleThrust(value, minimum):
        """ Function keep the PWM value above a threshold """
        if (value < minimum):
            value = minimum
        return value

    @eagerx.register.inputs(desired_thrust=Float32MultiArray, calculated_control=Float32MultiArray)
    @eagerx.register.outputs(pwm_signal=Float32MultiArray)
    def callback(self, t_n: float, desired_thrust: Msg, calculated_control: Msg):
        """ This node implements the power distribution node from the Crazyflie.
        It calculates the PWM signal per motor with the PWM signal difference per angle and the average thrust.

        :param desired_thrust: The desired average thrust, a value between [0, 65535]
        :param calculated_control: The PWM signal difference per angle (roll, pitch, yaw) in a range [-32767, 32767]

        :return pwm_signal: The PWM signal per motor send to the ForceController node
        """
        # define variables from inputs
        roll = calculated_control.msgs[-1].data[0]
        pitch = calculated_control.msgs[-1].data[1]
        yaw = calculated_control.msgs[-1].data[2]
        desired_thrust_input = desired_thrust.msgs[-1].data[0]

        roll = roll / 2
        pitch = pitch / 2
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

        new_pwm_signal = np.array([motorPower_m1, motorPower_m2, motorPower_m3, motorPower_m4])
        return dict(pwm_signal=Float32MultiArray(data=new_pwm_signal))


class ForceController(EngineNode):
    @staticmethod
    @register.spec("ForceController", EngineNode)
    def spec(
            spec: NodeSpec,
            name: str,
            rate: float,
            links: List[str] = None,
            process: Optional[int] = p.ENGINE,
            color: Optional[str] = "pink",
            mode: str = "external_force",  # = mode?
            force_target: List[float] = None,
    ):
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(ForceController)

        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = process
        spec.config.inputs = ["tick", "action", "orientation", "velocity"]
        spec.config.outputs = ["action_applied", "velocity_out"]

        # Set parameters, defined by the signature of cls.initialize(...)
        spec.config.links = links if isinstance(links, list) else []
        links = links if isinstance(links, list) else []
        spec.config.mode = mode
        spec.config.force_target = force_target if force_target else [0.0] * len(links)

    def initialize(self, links, mode, force_target):
        """Initializes the force controller node according to the spec."""
        self.obj_name = self.config["name"]
        assert self.process == p.ENGINE, (
            "Simulation node requires a reference to the simulator," " hence it must be launched in the Bridge process"
        )
        flag = self.obj_name in self.simulator["robots"]
        assert flag, f'Simulator object "{self.simulator}" is not compatible with this simulation node.'
        self.robot = self.simulator["robots"][self.obj_name]

        # If no links are provided, take baselink
        if len(links) == 0:
            for pb_name, part in self.robot.parts.items():
                bodyid, linkindex = part.get_bodyid_linkindex()
                if linkindex == -1:
                    links.append(pb_name)

        self.links = links
        self.mode = mode
        self.force_target = force_target
        self._p = self.simulator["client"]
        self.physics_client_id = self._p._client

        self.objectUniqueId = []
        self.linkIndex = []
        for _idx, pb_name in enumerate(links):
            objectid, linkindex = self.robot.parts[pb_name].get_bodyid_linkindex()
            self.objectUniqueId.append(objectid), self.linkIndex.append(linkindex)

        self._p.configureDebugVisualizer(self._p.COV_ENABLE_GUI, 0)
        self.force_cb = self._force_control(
            self._p,
            self.mode,
            self.objectUniqueId[0],
            self.linkIndex,
            [0, 0, 0]
        )
        self.torque_cb = self._force_control(
            self._p,
            "torque_control",
            self.objectUniqueId[0],
            self.linkIndex,
            [0, 0, 0]
        )

    @staticmethod
    def _force_control(p, mode, objectUniqueId, linkIndex, posObj):
        if mode == "external_force":  # also takes drag into account
            def cb(action, velocity, orientation):
                pwm = action[:4]
                forces = np.zeros(len(pwm))
                factor = 1.03  # To correct for hover PWM difference between ODE engine and pybullet engine 1.03
                offset = 1000  # To correct for hover PWM difference between ODE engine and pybullet engine 1000
                rotor_speed = np.zeros(len(pwm))
                drag_coeff_xy = 9.1785e-7
                drag_coeff_z = 10.311e-7
                drag_coefficients = np.array([drag_coeff_xy, drag_coeff_xy, drag_coeff_z])
                current_velocity = np.array(velocity.msgs[-1].data)  # x y z
                current_orientation = np.array(orientation.msgs[-1].data)
                rotation_matrix = np.array(p.getMatrixFromQuaternion(current_orientation)).reshape(3, 3)

                # Calculation of the force and rotor speed per motor. The formulas are obtained from Sytem Identification Crazyflie paper
                # source: https://doi.org/10.3929/ethz-b-000214143
                for idx, pwm in enumerate(pwm):
                    # PWM to forces formula with a pwm_factor to calibrate the model
                    forces[idx] = ((2.130295e-11) * (pwm * factor) ** 2 + (1.032633e-6) * (
                            pwm * factor) + 5.484560e-4)  # force in Newton (from Sytem Identification Crazyflie paper)

                    # PWM to forces formula with an offset to calibrate the model
                    forces[idx] = ((2.130295e-11) * (pwm + offset) ** 2 + (1.032633e-6) * (
                            pwm + offset) + 5.484560e-4)  # force in Newton (from Sytem Identification Crazyflie paper)

                    rotor_speed[
                        idx] = 0.04076521 * pwm + 380.8359  # rotor speed in rad/s (from Sytem Identification Crazyflie paper)

                total_force = np.array([0, 0, np.sum(forces)])

                # adding the drag
                rotor_speed_sum = np.sum(rotor_speed)
                drag_factor = - 1 * drag_coefficients * rotor_speed_sum * current_velocity
                drag = np.dot(rotation_matrix, drag_factor)
                total_force += drag

                # Apply the forces on the Crazyflie in Pybullet in the center of the Crazyflie. The moments of the forces are added later.
                p.applyExternalForce(
                    objectUniqueId=objectUniqueId,
                    linkIndex=linkIndex[0],
                    forceObj=total_force,
                    posObj=posObj,
                    flags=pybullet.LINK_FRAME,
                    physicsClientId=p._client,
                )
                return forces
        elif mode == "torque_control":
            # Calculates the moments caused by the forces and the torques caused by the motors.
            # based on coordinate system: https://www.bitcraze.io/documentation/system/platform/cf2-coordinate-system/
            # motor numbers: https://www.bitcraze.io/documentation/hardware/crazyflie_2_1/crazyflie_2_1-datasheet.pdf
            arm_1 = np.array([0.028, -0.028, 0])
            arm_2 = np.array([-0.028, -0.028, 0])
            arm_3 = np.array([-0.028, 0.028, 0])
            arm_4 = np.array([0.028, 0.028, 0])
            arms = np.array([arm_1, arm_2, arm_3, arm_4])

            def cb(forces):
                torques_pitchroll = np.zeros((len(forces), 3))
                torques_yaw = np.zeros((len(forces), 3))
                torques_yaw_direction = np.array([1, -1, 1, -1])
                total_torque = np.array([0, 0, 0])
                for idx, force in enumerate(forces):
                    # Calculate moments of the upward forces that causes the pitch and roll
                    torques_pitchroll[idx] = np.cross(arms[idx], np.array([0, 0, force])) # torque in Newton meter

                    # Calculate torques from the rotor rotation which causes the yaw. The formulas are obtained from Sytem Identification Crazyflie paper
                    # source: https://doi.org/10.3929/ethz-b-000214143
                    torque_yaw = 0.005964552 * force + 1.563383E-5 # torque in Newton meter (from Sytem Identification Crazyflie paper)
                    torques_yaw[idx] = np.array([0, 0, torques_yaw_direction[idx] * torque_yaw])

                    # Add up all torques en moments
                    total_torque = total_torque + torques_pitchroll[idx] + torques_yaw[idx]

                # Apply torque on the Crazyflie in Pybullet
                p.applyExternalTorque(
                    objectUniqueId=objectUniqueId,
                    linkIndex=linkIndex[0],
                    torqueObj=total_torque,
                    flags=pybullet.LINK_FRAME,
                    physicsClientId=p._client,
                )
        else:
            raise ValueError(f"Mode '{mode}' not recognized.")
        return cb

    @register.states()
    def reset(self):
        """This force controller is stateless, so nothing happens here."""
        pass

    @register.inputs(tick=UInt64, action=Float32MultiArray, orientation=Float32MultiArray, velocity=Float32MultiArray)
    @register.outputs(action_applied=Float32MultiArray, velocity_out=Float32MultiArray)
    def callback(self, t_n: float, tick: Msg, action: Msg, orientation: Msg, velocity: Msg):
        """ This node calculates the force and torque from the PWM signal per motor, the torque from the forces,
        the drag force and the yaw torque and sends it to the physics engine.

        :param action: The PWM signal per motor.
        :param orientation: The orientation in quaternions
        :param velocity: The velocity in meter per second

        :return: action_applied: The forces caused by the motors
        :return: velocity_out: The inputted velocity in meter per second
        """
        action_to_apply = action.msgs[-1]
        # action_applied.data = np.array([40000, 40000, 40000, 40000])  # debug
        total_force = self.force_cb(action_to_apply.data, velocity, orientation)
        self.torque_cb(total_force)
        # print('test', velocity.msgs[-1].data)

        return dict(action_applied=action_to_apply, velocity_out=Float32MultiArray(data=velocity))


class AccelerometerSensor(EngineNode):
    @staticmethod
    @register.spec("AccelerometerSensor", EngineNode)
    def spec(
            spec: NodeSpec,
            name: str,
            rate: float,
            process: Optional[int] = p.ENGINE,
            color: Optional[str] = "cyan",
    ):
        """A spec to create a LinkSensor node that provides sensor measurements for the specified set of links.

        :param spec: Holds the desired configuration.
        :param name: User specified node name.
        :param rate: Rate (Hz) at which the callback is called.
        :param links: List of links to be included in the measurements. Its order determines the ordering of the measurements.
        :param process: Process in which this node is launched. See :class:`~eagerx.core.constants.process` for all options.
        :param color: Specifies the color of logged messages & node color in the GUI.

        :return: NodeSpec
        """
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(AccelerometerSensor)

        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = process
        spec.config.inputs = ["tick", "input_velocity", "orientation"]
        spec.config.outputs = ["obs"]

    def initialize(self):
        # Direction of the gravitational acceleration as measured by the accelerometer
        self.gravity = np.array([0, 0, 9.81])

    @register.states()
    def reset(self):
        pass

    @register.inputs(tick=UInt64, input_velocity=Float32MultiArray, orientation=Float32MultiArray)
    @register.outputs(obs=Float32MultiArray)
    def callback(self, t_n: float, input_velocity: Msg, orientation: Msg, tick: Optional[Msg] = None):
        """ This node simulates an accelerometer.
        It calculates the derivative of the velocity (the acceleration) and adds the gravitational acceleration as measured by the acceleration.
        This means that the gravitational acceleration has to be aligned with the world frame upward direction.

        :param input_velocity: The velocity in meter per second
        :param orientation: The orientation in quaternions from Pybullet

        :return obs: The accelerometer measurement in meter per second send to the StateEstimator node

        """
        orientation_euler = pybullet.getEulerFromQuaternion(orientation.msgs[-1].data)
        roll = orientation_euler[0]
        pitch = -orientation_euler[1]

        rotation_matrix = np.array([[np.cos(-pitch), 0, -np.sin(-pitch)],
                                    [-np.sin(-roll) * np.sin(-pitch), np.cos(-roll), -np.sin(-roll) * np.cos(-pitch)],
                                    [np.cos(-roll) * np.sin(-pitch), np.sin(-roll), np.cos(-roll) * np.cos(-pitch)]])
        gravity_vector = rotation_matrix.dot(self.gravity)

        # get last and current velocity
        last = np.array(input_velocity.msgs[0].data)
        try:
            current = np.array(input_velocity.msgs[1].data)
        except:
            current = np.array([0, 0, 0])

        # calculate acceleration = dv/dt
        diff = (current - last) / (1 / self.rate)

        # Add the acceleration and the gravitational acceleration to calculate the accelerometer measurement
        acceleration = diff + gravity_vector

        return dict(obs=Float32MultiArray(data=acceleration))


class StateEstimator(eagerx.EngineNode):
    @staticmethod
    @eagerx.register.spec("StateEstimator", EngineNode)
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
        spec.config.outputs = ["orientation_state_estimator", "orientation_pybullet"]

        # Add space converters
        spec.inputs.angular_velocity.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                                  [0, 0, 0],
                                                                                  [3, 3, 3], dtype="float32")
        spec.inputs.acceleration.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                              [0, 0, 0],
                                                                              [3, 3, 3], dtype="float32")
        spec.inputs.orientation.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                             [-1, -1, -1, -1],
                                                                             [1, 1, 1, 1], dtype="float32")
        spec.outputs.orientation_pybullet.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                                       [-30, -30, -30],
                                                                                       [30, 30, 30], dtype="float32")
        spec.outputs.orientation_state_estimator.space_converter = eagerx.SpaceConverter.make("Space_Float32MultiArray",
                                                                                              [-30, -30, -30],
                                                                                              [30, 30, 30],
                                                                                              dtype="float32")

    def initialize(self):
        # initialize for Mahony's algorithm
        self.qw = 1.0
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0

        # For overall rate = 240 Hz (see paper on the Github for the calculations of the constants)
        self.twoKp = 2 * 0.4
        self.twoKi = 2 * 0.00048
        self.i = 0

        self.integralFBx = 0.0
        self.integralFBy = 0.0
        self.integralFBz = 0.0

        # initialize for Madgwick algorithm
        self.qw2 = 1.0
        self.qx2 = 0.0
        self.qy2 = 0.0
        self.qz2 = 0.0

        # Not tested
        self.beta = 0.01  # [2 * proportional gain (Kp)] = 0.01

    @eagerx.register.states()
    def reset(self):
        pass

    @eagerx.register.inputs(angular_velocity=Float32MultiArray, acceleration=Float32MultiArray,
                            orientation=Float32MultiArray)
    @eagerx.register.outputs(orientation_state_estimator=Float32MultiArray, orientation_pybullet=Float32MultiArray)
    def callback(self, t_n: float, angular_velocity: Msg, acceleration: Msg, orientation: Msg):
        """ This node implements the state estimator from the Crazyflie with two possible algorithms.
        1. Madgwick's implementation of Mahony's AHRS algorithm (default and used here).
        2. Implementation of Madgwick's IMU and AHRS algorithms.

        :param angular_velocity: The angular velocity in radians per second
        :param acceleration: The acceleration in meter per second (is normalised so unit doesn't really matter)
        :param orientation: The orientation from Pybullet in quaternions

        :return orientation_state_estimator: The orientation in euler angles calculated by the state estimator in degrees send to the AttitudePID node
        :return orientation_pybullet: The orientation in euler angles from Pybullet in degrees send to environment observations
        """
        angular_velocity = angular_velocity.msgs[-1].data
        acceleration = np.array(acceleration.msgs[-1].data)

        # Crazyflie default state estimator (Mahony's algorithm implemented by Madgwick)
        attitude_calculated = np.array(self.calculate_attitude_mahony(acceleration, angular_velocity))
        # Crazyflie Madgwick state estimator
        # attitude_calculated_madgwick = np.array(self.calculate_attitude_madgwick(acceleration, angular_velocity))

        # The pybullet output
        attitude = np.array(pybullet.getEulerFromQuaternion(orientation.msgs[-1].data)) * 180 / np.pi
        attitude_pitch_inverted = self.invert_pitch(attitude)

        # print(f"Pybullet  [qx, qy, qz, qw] = {orientation.msgs[-1].data}");
        # print("attitude from pybullet with inverted pitch = ", np.round(attitude_pitch_inverted, 3))
        # print("attitude from state estimator default      = ", np.round(attitude_calculated, 3))
        # print("attitude from state estimator Madgwick     = ", np.round(attitude_calculated_madgwick, 3))
        # print("=" * 80)
        return dict(orientation_state_estimator=Float32MultiArray(data=attitude_calculated),
                    orientation_pybullet=Float32MultiArray(data=attitude_pitch_inverted))

    @staticmethod
    def invert_pitch(attitude):
        return np.array([attitude[0], -attitude[1], attitude[2]])

    def calculate_attitude_mahony(self, acceleration, angular_velocity):
        """ Madgwick's implementation of Mahony's AHRS algorithm. The default state estimator in the Crazyflie
        Source: https://github.com/bitcraze/crazyflie-firmware/blob/2a282b575c2541d3f3b4552296952a6cc40bf5b5/src/modules/src/sensfusion6.c#L183-L251

        :param acceleration: The acceleration in meter per second (is normalised so unit doesn't really matter)
        :param angular_velocity: The angular velocity in radians per second

        :return [roll, pitch, yaw]: the estimated orientation in degrees
        """
        dt = (1 / self.rate)

        ax = acceleration[0]
        ay = acceleration[1]
        az = acceleration[2]
        gx = angular_velocity[0]  # rad/s
        gy = angular_velocity[1]  # rad/s
        gz = angular_velocity[2]  # rad/s

        if not ((ax == 0.0) and (ay == 0.0) and (az == 0.0)):
            recipNorm = 1 / np.sqrt(np.sum(acceleration ** 2))  # normalize acceleration
            ax *= recipNorm
            ay *= recipNorm
            az *= recipNorm

            # Estimated direction of gravity and vector perpendicular to magnetic flux
            halfvx = self.qx * self.qz - self.qw * self.qy
            halfvy = self.qw * self.qx + self.qy * self.qz
            halfvz = self.qw * self.qw - 0.5 + self.qz * self.qz

            # Error is sum of cross product between estimated and measured direction of gravity
            halfex = (ay * halfvz - az * halfvy)
            halfey = (az * halfvx - ax * halfvz)
            halfez = (ax * halfvy - ay * halfvx)

            if self.twoKi > 0.0:
                self.integralFBx += self.twoKi * halfex * dt  # integral error scaled by Ki
                self.integralFBy += self.twoKi * halfey * dt
                self.integralFBz += self.twoKi * halfez * dt
                gx += self.integralFBx  # apply integral feedback
                gy += self.integralFBy
                gz += self.integralFBz
            else:
                self.integralFBx = 0.0  # prevent integral windup
                self.integralFBy = 0.0
                self.integralFBz = 0.0

            # Apply proportional feedback
            gx += self.twoKp * halfex
            gy += self.twoKp * halfey
            gz += self.twoKp * halfez

        # Integrate rate of change of quaternion
        gx *= (0.5 * dt)  # pre-multiply common factors
        gy *= (0.5 * dt)
        gz *= (0.5 * dt)
        qa = self.qw
        qb = self.qx
        qc = self.qy
        self.qw += (-qb * gx - qc * gy - self.qz * gz)
        self.qx += (qa * gx + qc * gz - self.qz * gy)
        self.qy += (qa * gy - qb * gz + self.qz * gx)
        self.qz += (qa * gz + qb * gy - qc * gx)

        # Normalise quaternion
        recipNorm = 1 / np.sqrt(self.qw ** 2 + self.qx ** 2 + self.qy ** 2 + self.qz ** 2)
        self.qw *= recipNorm
        self.qx *= recipNorm
        self.qy *= recipNorm
        self.qz *= recipNorm

        # estimated gravity direction
        # https://github.com/bitcraze/crazyflie-firmware/blob/2a282b575c2541d3f3b4552296952a6cc40bf5b5/src/modules/src/sensfusion6.c#L305-L307
        gravX = 2 * (self.qx * self.qz - self.qw * self.qy)
        gravY = 2 * (self.qw * self.qx + self.qy * self.qz)
        gravZ = self.qw ** 2 - self.qx ** 2 - self.qy ** 2 + self.qz ** 2

        if gravX > 1:
            gravX = 1
        elif gravX < -1:
            gravX = -1

        # quaternion to euler for the crazyflie -> seems to be working; gives the same results as pybullet.getEulerfromQuaternion
        # https://github.com/bitcraze/crazyflie-firmware/blob/2a282b575c2541d3f3b4552296952a6cc40bf5b5/src/modules/src/sensfusion6.c#L272-L274
        yaw = np.arctan((2 * (self.qw * self.qz + self.qx * self.qy)) / (
                self.qw ** 2 + self.qx ** 2 - self.qy ** 2 - self.qz ** 2)) * 180 / np.pi
        pitch = np.arcsin(gravX) * 180 / np.pi  # here is the pitch inverted
        roll = np.arctan(gravY / gravZ) * 180 / np.pi

        return np.array([roll, pitch, yaw])

    def calculate_attitude_madgwick(self, acceleration, angular_velocity):
        """ Implementation of Madgwick's IMU and AHRS algorithms.
        Source: https://github.com/bitcraze/crazyflie-firmware/blob/2a282b575c2541d3f3b4552296952a6cc40bf5b5/src/modules/src/sensfusion6.c#L108-L172

        :param acceleration: The acceleration in meter per second (is normalised so unit doesn't really matter)
        :param angular_velocity: The angular velocity in radians per second

        :return [roll, pitch, yaw]: the estimated orientation in degrees
        """
        ax = acceleration[0]
        ay = acceleration[1]
        az = acceleration[2]
        gx = angular_velocity[0]  # rad/s
        gy = angular_velocity[1]  # rad/s
        gz = angular_velocity[2]  # rad/s
        dt = (1 / self.rate)

        qDot1 = 0.5 * (-self.qx2 * gx - self.qy2 * gy - self.qz2 * gz)
        qDot2 = 0.5 * (self.qw2 * gx + self.qy2 * gz - self.qz2 * gy)
        qDot3 = 0.5 * (self.qw2 * gy - self.qx2 * gz + self.qz2 * gx)
        qDot4 = 0.5 * (self.qw2 * gz + self.qx2 * gy - self.qy2 * gx)

        if not (ax == 0.0 and ay == 0.0 and az == 0):
            recipNorm = 1 / np.sqrt(ax ** 2 + ay ** 2 + az ** 2)
            ax *= recipNorm
            ay *= recipNorm
            az *= recipNorm

            _2qw = 2.0 * self.qw2
            _2qx = 2.0 * self.qx2
            _2qy = 2.0 * self.qy2
            _2qz = 2.0 * self.qz2
            _4qw = 4.0 * self.qw2
            _4qx = 4.0 * self.qx2
            _4qy = 4.0 * self.qy2
            _8qx = 8.0 * self.qx2
            _8qy = 8.0 * self.qy2
            qwqw = self.qw2 ** 2
            qxqx = self.qx2 ** 2
            qyqy = self.qy2 ** 2
            qzqz = self.qz2 ** 2

            # Gradient decent algorithm corrective step
            s0 = _4qw * qyqy + _2qy * ax + _4qw * qxqx - _2qx * ay
            s1 = _4qx * qzqz - _2qz * ax + 4.0 * qwqw * self.qx2 - _2qw * ay - _4qx + _8qx * qxqx + _8qx * qyqy + _4qx * az
            s2 = 4.0 * qwqw * self.qy2 + _2qw * ax + _4qy * qzqz - _2qz * ay - _4qy + _8qy * qxqx + _8qy * qyqy + _4qy * az
            s3 = 4.0 * qxqx * self.qz2 - _2qx * ax + 4.0 * qyqy * self.qz2 - _2qy * ay
            if not (s0 == 0.0 and s1 == 0 and s2 == 0 and s3 == 0):
                recipNorm = 1 / np.sqrt(s0 ** 2 + s1 ** 2 + s2 ** 2 + s3 ** 2)
                s0 *= recipNorm
                s1 *= recipNorm
                s2 *= recipNorm
                s3 *= recipNorm

            # Apply feedback step
            qDot1 -= self.beta * s0
            qDot2 -= self.beta * s1
            qDot3 -= self.beta * s2
            qDot4 -= self.beta * s3

        self.qw2 += qDot1 * dt
        self.qx2 += qDot2 * dt
        self.qy2 += qDot3 * dt
        self.qz2 += qDot4 * dt

        recipNorm = 1 / np.sqrt(self.qw2 ** 2 + self.qx2 ** 2 + self.qy2 ** 2 + self.qz2 ** 2)
        self.qw2 *= recipNorm
        self.qx2 *= recipNorm
        self.qy2 *= recipNorm
        self.qz2 *= recipNorm

        # estimated gravity direction
        # https://github.com/bitcraze/crazyflie-firmware/blob/2a282b575c2541d3f3b4552296952a6cc40bf5b5/src/modules/src/sensfusion6.c#L305-L307
        gravX = 2 * (self.qx2 * self.qz2 - self.qw2 * self.qy2)
        gravY = 2 * (self.qw2 * self.qx2 + self.qy2 * self.qz2)
        gravZ = self.qw2 ** 2 - self.qx2 ** 2 - self.qy2 ** 2 + self.qz2 ** 2
        if gravX > 1:
            gravX = 1
        elif gravX < -1:
            gravX = -1

        # quaternion to euler for the crazyflie -> seems to be working; gives the same results as pybullet.getEulerfromQuaternion
        # https://github.com/bitcraze/crazyflie-firmware/blob/2a282b575c2541d3f3b4552296952a6cc40bf5b5/src/modules/src/sensfusion6.c#L272-L274
        yaw = np.arctan((2 * (self.qw2 * self.qz2 + self.qx2 * self.qy2)) / (
                self.qw2 ** 2 + self.qx2 ** 2 - self.qy2 ** 2 - self.qz2 ** 2)) * 180 / np.pi
        pitch = np.arcsin(gravX) * 180 / np.pi  # here is the pitch inverted
        roll = np.arctan(gravY / gravZ) * 180 / np.pi

        return np.array([roll, pitch, yaw])
