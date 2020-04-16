"""Define constants
"""
import numpy as np


INCH = 0.0254 # meters
BOUND = np.array([[2*INCH, 2*INCH], [-2*INCH, -2*INCH]]) # fixed boundary
EE_RADIUS = 0*INCH # end-effector radius
EE_LENGTH = 6*INCH # end-effector length
Z_TARGET = 0*INCH # target z position
OBS_HEIGHT = 8*INCH # obstacle height