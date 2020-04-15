""" Plot trajectory in 2D
"""
import matplotlib.pyplot as plt
import argparse
from scipy.io import loadmat
from constants import INCH, BOUND, EE_RADIUS
from robot import Robot
import pybullet as p


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

    # Plot result in inches
    fig = plt.figure(figsize=(6,6))
    plt.rcParams.update({"font.size": 12})
    ax = fig.add_subplot(111, aspect="equal")
    ax.set_xlabel("x (inch)")
    ax.set_ylabel("y (inch)")
    # Plot boundary
    plt.xlim(min(bound[:, 0])/INCH, max(bound[:, 0])/INCH)
    plt.ylim(min(bound[:, 1])/INCH, max(bound[:, 1])/INCH)
    # Plot obstacles
    for ob in obs:
        circle = plt.Circle(ob/INCH, radius=ob[2]/INCH)
        ax.add_artist(circle)

    # Create the robot
    p.connect(p.DIRECT)
    arm = Robot(p, "models/2dplannar.urdf")

    # Plot trajectory
    N = q.shape[1]
    for i in range(N):
        arm.set_to(q[:2, i])
        print(arm.endpoint_pos()[:2]/INCH)

        # Plot trajectory
        pos = arm.endpoint_pos()
        circle = plt.Circle(pos[:2]/INCH, radius=0.05, color="green")
        ax.add_artist(circle)

    plt.show()