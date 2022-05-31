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
