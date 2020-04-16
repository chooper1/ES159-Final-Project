"""Run a trajectory of the robot in the simulation environment
"""
import pybullet as p
import numpy as np
from scipy.io import loadmat, savemat
from robot import Robot
from environment import Environment
from constants import INCH, BOUND, EE_RADIUS, Z_TARGET
import time
import argparse


if __name__ == "__main__":
    # Parse commandline arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("inputs", type=str, nargs=2)
    inputs = parser.parse_args().inputs

    # Load inputs
    start_pos, goal_pos = loadmat(inputs[0])["pos"].T * INCH
    obs = loadmat(inputs[0])["obs"].T * INCH
    q = loadmat(inputs[1])["q"] # in degrees
    bound = BOUND

    # Configure default display
    clid = p.connect(p.SHARED_MEMORY)
    if clid < 0:
        p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.resetDebugVisualizerCamera(0.6, 0.0, -85.0, (0*INCH, 0.0, 0*INCH))

    # Set up environment
    env = Environment(p, gravity=0)
    env.add_object("models/plane.urdf")
    env.set_boundary(bound)
    env.add_cylinders(obs)

    # Initialize start and goal position
    start_pos = [*start_pos, Z_TARGET]
    goal_pos = [*goal_pos, Z_TARGET]

    # Create the robot
    catalyst = Robot(p, "models/2dplannar.urdf")

    # Pause for 2 seconds before executing trajectory
    for i in range(60*2):
        time.sleep(1/60.)

    # Run simulation
    p.setRealTimeSimulation(0)
    N = q.shape[1]
    while True:
        # Set initial configuration and check it
        catalyst.set_to(q[:2, 0])
        print(catalyst.endpoint_pos())
        print(start_pos)
        if not np.allclose(catalyst.endpoint_pos(), start_pos, atol=1e-4):
            print("Error: initial end-effector position not at start pos.")
        
            exit()
        # Run through trajectory
        for i in range(N):
            catalyst.set_to(q[:2, i])
            # catalyst.control_to(q[:4, i])
            p.stepSimulation()
            # time.sleep(1/60)
            time.sleep(1/60)
            if env.has_collision(catalyst):
                print("Error: collision!")

        # Check end position
        if not np.allclose(catalyst.endpoint_pos(), goal_pos, atol=1e-4):
            print("Error: end end-effector position not at goal pos.")
            exit()

    p.disconnect()
