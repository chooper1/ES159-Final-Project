"""Implement the robot class
"""
import pybullet
import numpy as np
from constants import INCH, EE_LENGTH
from scipy.spatial.transform import Rotation


class Robot:
    """Robot arm class for use with pybullet.
    """
    def __init__(self, p, urdf_path, base_pos=[0, 0, 0],
                 base_orientaion=[0, 0, 0, 1]):
        """Initialize the robot model.
        Args:
            p: pybullet library object
            urdf_path: string of the path to the robot urdf file
            base_pos: the robot base position
            base_orientation: the robot base orientation as a quaternion
        """
        self.p = p
        self.id = self.p.loadURDF(urdf_path, base_pos, base_orientaion,
                    useFixedBase=1,
                    flags=pybullet.URDF_MERGE_FIXED_LINKS |
                          pybullet.URDF_USE_SELF_COLLISION)
        self.num_joints = self.p.getNumJoints(self.id)
        self.ee_l = EE_LENGTH # End-effector

    def control_to(self, joint_values):
        """Control the robot to target joint values.
        Args:
            joint_values: a list of target joint values in degrees
        """
        rad_values = np.deg2rad(joint_values)
        self.p.setJointMotorControlArray(
                bodyIndex=self.id,
                jointIndices=range(self.num_joints),
                controlMode=pybullet.POSITION_CONTROL,
                targetPositions=rad_values
                )

    def set_to(self, joint_values):
        """Set the robot to target joint values
        Args:
            joint_values: a list of joint values to set to
        """
        rad_values = np.deg2rad(joint_values)
        for i in range(self.num_joints):
            self.p.resetJointState(self.id, i, rad_values[i])

    def endpoint_pos(self):
        """Get the end-effector's endpoint position
        """
        state = self.p.getLinkState(self.id, self.num_joints-1)
        pos = state[0]
        z_dir = Rotation(state[1]).as_matrix()[:, -1]
        return pos + self.ee_l/2*z_dir
