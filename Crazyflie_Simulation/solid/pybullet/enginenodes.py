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
        # spec.initialize(ForceController)

        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = process
        spec.config.inputs = ["tick", "action"]
        spec.config.outputs = ["action_applied"]

        # Set parameters, defined by the signature of cls.initialize(...)
        spec.config.links = links if isinstance(links, list) else []
        links = links if isinstance(links, list) else []
        spec.config.mode = mode
        spec.config.force_target = force_target if force_target else [0.0] * len(links)

    def initialize(self, links, mode, force_target):
        """Initializes the link sensor node according to the spec."""
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
        if mode == "external_force":
            def cb(action):
                pwm = action[:4]
                forces = np.zeros(len(pwm))
                for idx, pwm in enumerate(pwm):
                    forces[idx] = (2.130295e-11) * (pwm) ** 2 + (1.032633e-6) * pwm + 5.484560e-4
                total_force = np.array([0, 0, np.sum(forces)])

                # this acceleration differs from acceleration calculated in Accelerometer. Only works when flying straight up
                # accel = (total_force/0.027) + np.array([0, 0, -9.81]) # debug
                # print("="*50)
                # print("accel from applied force", accel) # debug

                # print(f"Force: {total_force}")  # debug
                p.applyExternalForce(
                    objectUniqueId=objectUniqueId,
                    linkIndex=linkIndex[0],
                    forceObj=total_force,
                    posObj=posObj,
                    flags=pybullet.LINK_FRAME,
                    physicsClientId=p._client,
                )
                # p.applyExternalForce(
                #     objectUniqueId=1,
                #     linkIndex=-1,
                #     forceObj=(0, 0, 0.04 * 9.81),
                #     posObj=(0, 0, 0),
                #     flags=p.LINK_FRAME,
                # )
        elif mode == "torque_control":
            # based on coordinate system: https://www.bitcraze.io/documentation/system/platform/cf2-coordinate-system/
            # motor numbers: https://www.bitcraze.io/documentation/hardware/crazyflie_2_1/crazyflie_2_1-datasheet.pdf
            arm_1 = np.array([0.028, -0.028, 0])
            arm_2 = np.array([-0.028, -0.028, 0])
            arm_3 = np.array([-0.028, 0.028, 0])
            arm_4 = np.array([0.028, 0.028, 0])
            arms = np.array([arm_1, arm_2, arm_3, arm_4])

            def cb(action):
                pwm = action[:4]
                forces = np.zeros(len(pwm))
                torques = np.zeros((len(pwm), 3))
                total_torque = np.array([0, 0, 0])
                for idx, pwm in enumerate(pwm):
                    force = (2.130295e-11) * (pwm) ** 2 + (1.032633e-6) * pwm + 5.484560e-4
                    forces[idx] = force
                    torques[idx] = np.cross(arms[idx], np.array([0, 0, force]))
                    total_torque = total_torque + torques[idx]
                # print(f"Torque: {total_torque}")  # debug
                p.applyExternalTorque(
                    objectUniqueId=objectUniqueId,
                    linkIndex=linkIndex[0],
                    torqueObj=total_torque,
                    flags=pybullet.LINK_FRAME,
                    physicsClientId=p._client,
                )
        else:
            raise ValueError(f"Mode '{mode}' not recognized.")
        # todo: add yaw torque
        return cb

    @register.states()
    def reset(self):
        """This force controller is stateless, so nothing happens here."""
        pass

    @register.inputs(tick=UInt64, action=Float32MultiArray)
    @register.outputs(action_applied=Float32MultiArray)
    def callback(self, t_n: float, tick: Msg, action: Msg):
        """Produces a link sensor measurement called `obs`.

        The measurement is published at the specified rate * real_time_factor.

        Input `tick` ensures that this node is I/O synchronized with the simulator."""
        action_applied = action.msgs[-1]
        # action_applied.data = np.array([40000, 40000, 40000, 40000])  # debug
        self.force_cb(action_applied.data)
        self.torque_cb(action_applied.data)

        return dict(action_applied=action_applied)

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
        :param mode: Available: `position`, `orientation`, `velocity`, and `angular_vel`
        :return: NodeSpec
        """
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(AccelerometerSensor)

        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = process
        spec.config.inputs = ["tick", "input_velocity"]
        spec.config.outputs = ["obs"]

    def initialize(self):
        pass

    @register.states()
    def reset(self):
        """This link sensor is stateless, so nothing happens here."""
        pass

    @register.inputs(tick=UInt64, input_velocity=Float32MultiArray)
    @register.outputs(obs=Float32MultiArray)
    def callback(self, t_n: float, input_velocity: Msg, tick: Optional[Msg] = None):
        """Produces a link sensor measurement called `obs`.

        The measurement is published at the specified rate * real_time_factor.

        Input `tick` ensures that this node is I/O synchronized with the simulator."""
        # get last and current velocity
        last = np.array(input_velocity.msgs[0].data)
        # print("last", last) # debug
        try:
            current = np.array(input_velocity.msgs[1].data)
        except:
            current = np.array([0, 0, 0])
        # print("current", current) # debug

        # calculate acceleration dy/dt
        # todo: This acceleration is not exactly the same as calculated from the applied force in external_force
        diff = (current - last) / (1 / self.rate)
        # print("acceleration", diff)       # debug
        # print("=" * 50)                   # debug
        return dict(obs=Float32MultiArray(data=diff))

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
        # attitude = orientation.msgs[-1].data
        attitude = [0,0,0,1]
        return dict(orientation=Float32MultiArray(data=attitude))

class FloatMultiArrayOutput(EngineNode):
    @staticmethod
    @register.spec("FloatMultiArrayOutput", EngineNode)
    def spec(
            spec,
            name: str,
            rate: float,
            idx: Optional[list] = [0],
            process: Optional[int] = p.ENVIRONMENT,
            color: Optional[str] = "cyan",
    ):
        """
        FloatOutput spec
        :param idx: index of the value of interest from the array.
        """
        # Performs all the steps to fill-in the params with registered info about all functions.

        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = process
        spec.config.inputs = ["observation_array"]
        spec.config.outputs = ["observation"]

        # Custom node params
        spec.config.idx = idx

    def initialize(self, idx):
        self.obj_name = self.config["name"]
        self.idx = idx

    @register.states()
    def reset(self):
        pass

    @register.inputs(observation_array=Float32MultiArray)
    @register.outputs(observation=Float32MultiArray)
    def callback(self, t_n: float, observation_array: Optional[Msg] = None):
        data = len(self.idx) * [0]
        for idx, _data in enumerate(self.idx):
            data[idx] = observation_array.msgs[-1].data[_data]
        # if statement to add yaw since Jacob didn't use that
        if self.idx[0] == 6 and self.idx[1] == 7:
            data.append(0)
        return dict(observation=Float32MultiArray(data=data))


class OdeMultiInput(EngineNode):
    @staticmethod
    @register.spec("OdeMultiInput", EngineNode)
    def spec(
            spec,
            name: str,
            rate: float,
            default_action: List,
            process: Optional[int] = p.ENGINE,
            color: Optional[str] = "green",
    ):
        """OdeInput spec"""
        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = process
        spec.config.inputs = ["tick", "commanded_thrust", "commanded_attitude"]
        spec.config.outputs = ["action_applied"]

        # Set custom node params
        spec.config.default_action = default_action

    def initialize(self, default_action):
        # We will probably use self.simulator[self.obj_name] in callback & reset.
        assert (
                self.process == p.ENGINE
        ), "Simulation node requires a reference to the simulator, hence it must be launched in the Engine process"
        self.obj_name = self.config["name"]
        self.default_action = np.array(default_action)

    @register.states()
    def reset(self):
        self.simulator[self.obj_name]["input"] = np.squeeze(np.array(self.default_action))

    @register.inputs(tick=UInt64, commanded_thrust=Float32MultiArray, commanded_attitude=Float32MultiArray)
    @register.outputs(action_applied=Float32MultiArray)
    def callback(
            self,
            t_n: float,
            tick: Optional[Msg] = None,
            commanded_thrust: Optional[Float32MultiArray] = None,
            commanded_attitude: Optional[Float32MultiArray] = None,
    ):
        assert isinstance(self.simulator[self.obj_name], dict), (
                'Simulator object "%s" is not compatible with this simulation node..' % self.simulator[self.obj_name]
        )
        u = [np.squeeze(commanded_thrust.msgs[-1].data), np.squeeze(commanded_attitude.msgs[-1].data[0]),
             np.squeeze(commanded_attitude.msgs[-1].data[1])]
        action_applied = [commanded_thrust.msgs[-1], commanded_attitude.msgs[-1]]
        # Set action in simulator for next step.
        self.simulator[self.obj_name]["input"] = u

        # Send action that has been applied.
        # return dict(action_applied=action_applied)
        return dict(action_applied=Float32MultiArray(data=action_applied))

# class LinkSensor(EngineNode):
#     @staticmethod
#     @register.spec("LinkSensor", EngineNode)
#     def spec(
#         spec: NodeSpec,
#         name: str,
#         rate: float,
#         links: List[str] = None,
#         process: Optional[int] = p.ENGINE,
#         color: Optional[str] = "cyan",
#         mode: str = "position",
#     ):
#         """A spec to create a LinkSensor node that provides sensor measurements for the specified set of links.
#
#         :param spec: Holds the desired configuration.
#         :param name: User specified node name.
#         :param rate: Rate (Hz) at which the callback is called.
#         :param links: List of links to be included in the measurements. Its order determines the ordering of the measurements.
#         :param process: Process in which this node is launched. See :class:`~eagerx.core.constants.process` for all options.
#         :param color: Specifies the color of logged messages & node color in the GUI.
#         :param mode: Available: `position`, `orientation`, `velocity`, and `angular_vel`
#         :return: NodeSpec
#         """
#         # Performs all the steps to fill-in the params with registered info about all functions.
#         spec.initialize(LinkSensor)
#
#         # Modify default node params
#         spec.config.name = name
#         spec.config.rate = rate
#         spec.config.process = process
#         spec.config.inputs = ["tick"]
#         spec.config.outputs = ["obs"]
#
#         # Set parameters, defined by the signature of cls.initialize(...)
#         spec.config.links = links if isinstance(links, list) else []
#         spec.config.mode = mode
#
#     def initialize(self, links, mode):
#         """Initializes the link sensor node according to the spec."""
#         self.obj_name = self.config["name"]
#         assert self.process == p.ENGINE, (
#             "Simulation node requires a reference to the simulator," " hence it must be launched in the engine process"
#         )
#         flag = self.obj_name in self.simulator["robots"]
#         assert flag, f'Simulator object "{self.simulator}" is not compatible with this simulation node.'
#         self.robot = self.simulator["robots"][self.obj_name]
#         # If no links are provided, take baselink
#         if len(links) == 0:
#             for pb_name, part in self.robot.parts.items():
#                 bodyid, linkindex = part.get_bodyid_linkindex()
#                 if linkindex == -1:
#                     links.append(pb_name)
#         self.links = links
#         self.mode = mode
#         self._p = self.simulator["client"]
#         self.physics_client_id = self._p._client
#         self.link_cb = self._link_measurement(self._p, self.mode, self.robot, links)
#
#     @register.states()
#     def reset(self):
#         """This link sensor is stateless, so nothing happens here."""
#         pass
#
#     @register.inputs(tick=UInt64)
#     @register.outputs(obs=Float32MultiArray)
#     def callback(self, t_n: float, tick: Optional[Msg] = None):
#         """Produces a link sensor measurement called `obs`.
#
#         The measurement is published at the specified rate * real_time_factor.
#
#         Input `tick` ensures that this node is I/O synchronized with the simulator."""
#         obs = self.link_cb()
#         return dict(obs=Float32MultiArray(data=obs))
#
#     @staticmethod
#     def _link_measurement(p, mode, robot, links):
#         if mode == "position":  # (x, y, z)
#
#             def cb():
#                 obs = []
#                 for pb_name in links:
#                     obs += robot.parts[pb_name].get_position().tolist()
#                 return obs
#
#         elif mode == "orientation":  # (x, y, z, w)
#
#             def cb():
#                 obs = []
#                 for pb_name in links:
#                     obs += robot.parts[pb_name].get_orientation().tolist()
#                 return obs
#
#         elif mode == "velocity":  # (vx, vy, vz)
#
#             def cb():
#                 obs = []
#                 for pb_name in links:
#                     vel, _ = robot.parts[pb_name].speed()
#                     obs += vel.tolist()
#                 return obs
#
#         elif mode == "angular_vel":  # (vx, vy, vz)
#
#             def cb():
#                 obs = []
#                 for pb_name in links:
#                     _, rate = robot.parts[pb_name].speed()
#                     obs += rate.tolist()
#                 return obs
#
#         else:
#             raise ValueError(f"Mode '{mode}' not recognized.")
#         return cb


# class JointController(EngineNode):
#     @staticmethod
#     @register.spec("JointController", EngineNode)
#     def spec(
#         spec: NodeSpec,
#         name: str,
#         rate: float,
#         joints: List[str],
#         process: Optional[int] = p.ENGINE,
#         color: Optional[str] = "green",
#         mode: str = "position_control",
#         vel_target: List[float] = None,
#         pos_gain: List[float] = None,
#         vel_gain: List[float] = None,
#         max_force: List[float] = None,
#     ):
#         """A spec to create a JointController node that controls a set of joints.
#
#         For more info on `vel_target`, `pos_gain`, and `vel_gain`, see `setJointMotorControlMultiDofArray` in
#         https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#
#
#         :param spec: Holds the desired configuration in a Spec object.
#         :param name: User specified node name.
#         :param rate: Rate (Hz) at which the callback is called.
#         :param joints: List of controlled joints. Its order determines the ordering of the applied commands.
#         :param process: Process in which this node is launched. See :class:`~eagerx.core.constants.process` for all options.
#         :param color: Specifies the color of logged messages & node color in the GUI.
#         :param mode: Available: `position_control`, `velocity_control`, `pd_control`, and `torque_control`.
#         :param vel_target: The desired velocity. Ordering according to `joints`.
#         :param pos_gain: Position gain. Ordering according to `joints`.
#         :param vel_gain: Velocity gain. Ordering according to `joints`.
#         :param max_force: Maximum force when mode in [`position_control`, `velocity_control`, `pd_control`]. Ordering
#                           according to `joints`.
#         :return: NodeSpec
#         """
#         # Performs all the steps to fill-in the params with registered info about all functions.
#         spec.initialize(JointController)
#
#         # Modify default node params
#         spec.config.name = name
#         spec.config.rate = rate
#         spec.config.process = process
#         spec.config.inputs = ["tick", "action"]
#         spec.config.outputs = ["action_applied"]
#
#         # Set parameters, defined by the signature of cls.initialize(...)
#         spec.config.joints = joints
#         spec.config.mode = mode
#         spec.config.vel_target = vel_target if vel_target else [0.0] * len(joints)
#         spec.config.pos_gain = pos_gain if pos_gain else [0.2] * len(joints)
#         spec.config.vel_gain = vel_gain if vel_gain else [0.2] * len(joints)
#         spec.config.max_force = max_force if max_force else [999.0] * len(joints)
#
#     def initialize(self, joints, mode, vel_target, pos_gain, vel_gain, max_force):
#         """Initializes the joint controller node according to the spec."""
#         # We will probably use self.simulator[self.obj_name] in callback & reset.
#         self.obj_name = self.config["name"]
#         assert self.process == p.ENGINE, (
#             "Simulation node requires a reference to the simulator," " hence it must be launched in the Engine process"
#         )
#         flag = self.obj_name in self.simulator["robots"]
#         assert flag, f'Simulator object "{self.simulator}" is not compatible with this simulation node.'
#         self.joints = joints
#         self.mode = mode
#         self.vel_target = vel_target
#         self.pos_gain = pos_gain
#         self.vel_gain = vel_gain
#         self.max_force = max_force
#         self.robot = self.simulator["robots"][self.obj_name]
#         self._p = self.simulator["client"]
#         self.physics_client_id = self._p._client
#
#         self.bodyUniqueId = []
#         self.jointIndices = []
#         for _idx, pb_name in enumerate(joints):
#             bodyid, jointindex = self.robot.jdict[pb_name].get_bodyid_jointindex()
#             self.bodyUniqueId.append(bodyid), self.jointIndices.append(jointindex)
#
#         self.joint_cb = self._joint_control(
#             self._p,
#             self.mode,
#             self.bodyUniqueId[0],
#             self.jointIndices,
#             self.pos_gain,
#             self.vel_gain,
#             self.vel_target,
#             self.max_force,
#         )
#
#     @register.states()
#     def reset(self):
#         """This joint controller is stateless, so nothing happens here."""
#         pass
#
#     @register.inputs(tick=UInt64, action=Float32MultiArray)
#     @register.outputs(action_applied=Float32MultiArray)
#     def callback(
#         self,
#         t_n: float,
#         tick: Optional[Msg] = None,
#         action: Optional[Msg] = None,
#     ):
#         """Sets the most recently received `action` in the pybullet joint controller.
#
#         The action is set at the specified rate * real_time_factor.
#
#         The output `action_applied` is the action that was set. If the input `action` comes in at a higher rate than
#         this node's rate, `action_applied` may be differnt as only the most recently received `action` is set.
#
#         Input `tick` ensures that this node is I/O synchronized with the simulator."""
#         # Set action in pybullet
#         self.joint_cb(action.msgs[-1].data)
#         # Send action that has been applied.
#         return dict(action_applied=action.msgs[-1])
#
#     @staticmethod
#     def _joint_control(p, mode, bodyUniqueId, jointIndices, pos_gain, vel_gain, vel_target, max_force):
#         if mode == "position_control":
#
#             def cb(action):
#                 return p.setJointMotorControlArray(
#                     bodyUniqueId=bodyUniqueId,
#                     jointIndices=jointIndices,
#                     controlMode=pybullet.POSITION_CONTROL,
#                     targetPositions=action,
#                     targetVelocities=vel_target,
#                     positionGains=pos_gain,
#                     velocityGains=vel_gain,
#                     forces=max_force,
#                     physicsClientId=p._client,
#                 )
#
#         elif mode == "velocity_control":
#
#             def cb(action):
#                 return p.setJointMotorControlArray(
#                     bodyUniqueId=bodyUniqueId,
#                     jointIndices=jointIndices,
#                     controlMode=pybullet.VELOCITY_CONTROL,
#                     targetVelocities=action,
#                     positionGains=pos_gain,
#                     velocityGains=vel_gain,
#                     forces=max_force,
#                     physicsClientId=p._client,
#                 )
#
#         elif mode == "pd_control":
#
#             def cb(action):
#                 return p.setJointMotorControlArray(
#                     bodyUniqueId=bodyUniqueId,
#                     jointIndices=jointIndices,
#                     controlMode=pybullet.PD_CONTROL,
#                     targetVelocities=action,
#                     positionGains=pos_gain,
#                     velocityGains=vel_gain,
#                     forces=max_force,
#                     physicsClientId=p._client,
#                 )
#
#         elif mode == "torque_control":
#
#             def cb(action):
#                 return p.setJointMotorControlArray(
#                     bodyUniqueId=bodyUniqueId,
#                     jointIndices=jointIndices,
#                     controlMode=pybullet.TORQUE_CONTROL,
#                     forces=action,
#                     positionGains=pos_gain,
#                     velocityGains=vel_gain,
#                     physicsClientId=p._client,
#                 )
#
#         else:
#             raise ValueError(f"Mode '{mode}' not recognized.")
#         return cb
