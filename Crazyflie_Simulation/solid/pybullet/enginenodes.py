from typing import Optional, List
from scipy.spatial.transform import Rotation
import pybullet

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


class ForceController(EngineNode):
    @staticmethod
    @register.spec("ForceController", EngineNode)
    def spec(
            spec: NodeSpec,
            name: str,
            rate: float,
            links: List[str] = None,
            process: Optional[int] = p.BRIDGE,
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
        assert self.process == p.BRIDGE, (
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

# class LinkSensor(EngineNode):
#     @staticmethod
#     @register.spec("LinkSensor", EngineNode)
#     def spec(
#         spec: NodeSpec,
#         name: str,
#         rate: float,
#         links: List[str] = None,
#         process: Optional[int] = p.BRIDGE,
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
#         assert self.process == p.BRIDGE, (
#             "Simulation node requires a reference to the simulator," " hence it must be launched in the Bridge process"
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
#
#
# class JointController(EngineNode):
#     @staticmethod
#     @register.spec("JointController", EngineNode)
#     def spec(
#         spec: NodeSpec,
#         name: str,
#         rate: float,
#         joints: List[str],
#         process: Optional[int] = p.BRIDGE,
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
#         assert self.process == p.BRIDGE, (
#             "Simulation node requires a reference to the simulator," " hence it must be launched in the Bridge process"
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
