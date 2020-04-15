"""Implement the environment class
"""
import pybullet
from constants import INCH, OBS_HEIGHT
import numpy as np


class Boundary:
    def __init__(self, opposite_corners):
        """Initialize from opposite corners
        Args:
            opposite_corners: a np array where first row is (x, y) of
                bottom left corner, and second row is (x, y) of upper right
                corner of a rectangular boundary
        """
        x0, y0 = opposite_corners[0]
        x1, y1 = opposite_corners[1]
        self.x_min = min(x0, x1)
        self.x_max = max(x0, x1)
        self.y_min = min(y0, y1)
        self.y_max = max(y0, y1)

    @property
    def half_x(self):
        return (self.x_max-self.x_min)/2

    @property
    def half_y(self):
        return (self.y_max-self.y_min)/2

    @property
    def x_center(self):
        return self.x_min + self.half_x

    @property
    def y_center(self):
        return self.y_min + self.half_y

    def sample(self):
        return (np.random.uniform(self.x_min, self.x_max),
                np.random.uniform(self.y_min, self.y_max))


class Environment:
    """The environment within which the robot would act
    """
    def __init__(self, p, gravity=-9.8):
        """Initialize the environment.
        Args:
            p: pybullet library object
            gravity: int, the gravity value of the world, in z-axis direction
        """
        self.p = p
        self.p.setGravity(0, 0, gravity)
        self.obs_ids = []

    def set_boundary(self, opposite_corners):
        """Sets the boundary of the workspace
        Args:
            opposite_corners: a np array where first row is (x, y) of
                bottom left corner, and second row is (x, y) of upper right
                corner of a rectangular boundary
        """
        self.ws_bound = Boundary(opposite_corners)

        half_x = self.ws_bound.half_x
        half_y = self.ws_bound.half_y
        height = OBS_HEIGHT
        width = 0.01
        z = 1*INCH
        mass = -1 # for fixed objects

        # Create boundaries
        col_top_bottom_box_id = self.p.createCollisionShape(
            pybullet.GEOM_BOX,
            halfExtents=[half_x, width/2, height/2])
        self.obs_ids.append(
            self.p.createMultiBody(mass, col_top_bottom_box_id, -1,
                (self.ws_bound.x_center, self.ws_bound.y_min-width/2, z)))

        self.obs_ids.append(
            self.p.createMultiBody(mass, col_top_bottom_box_id, -1,
                (self.ws_bound.x_center, self.ws_bound.y_max+width/2, z)))

        col_left_right_box_id = self.p.createCollisionShape(
            pybullet.GEOM_BOX,
            halfExtents=[width/2, half_y, height/2])
        self.obs_ids.append(
            self.p.createMultiBody(mass, col_left_right_box_id, -1,
                (self.ws_bound.x_min-width/2, self.ws_bound.y_center, z)))

        self.obs_ids.append(
            self.p.createMultiBody(mass, col_left_right_box_id, -1,
                (self.ws_bound.x_max+width/2, self.ws_bound.y_center, z)))


    def add_object(self, urdf_path, pos=[0, 0, 0], orientation=[0, 0, 0, 1]):
        """Add an object to the environment
        Args:
            urdf_path: string of the path to the object urdf
            pos: initial position of the object
            orientation: initial orientation of the object as a quaternion
        """
        self.p.loadURDF(urdf_path, pos, orientation)

    def add_cylinders(self, positions):
        """Add cylinder obstacles at positions.
        Args:
            positions: a list of (x, y) values of the positions
        """
        mass = -1
        height = OBS_HEIGHT
        z = 0
        for pos in positions:
            x, y, rad = pos
            col_cylinder_id = self.p.createCollisionShape(
                pybullet.GEOM_CYLINDER,
                radius=rad,
                height=0)
            self.obs_ids.append(
                self.p.createMultiBody(mass, col_cylinder_id, -1, (x, y, z)))

    def has_collision(self, robot):
        """Check if any obstacle has collision with robot.
        Args:
            robot: a Robot object
        """
        for obs_id in self.obs_ids:
            contact_data = self.p.getContactPoints(obs_id)
            for datum in contact_data:
                other_id = datum[2]
                distance = datum[8]
                if other_id == robot.id and distance <= 0:
                    return True
        return False
