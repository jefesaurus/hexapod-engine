import matplotlib as mpl
import mpl_toolkits.mplot3d.axes3d as p3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from math import pi,cos,sin,sqrt
from random import random

from model import *


class SuperModel:
    fig = plt.figure()
    ax = p3.Axes3D(fig)
    chassis = animatedChassis()
    lines = []

    ax.set_xlim3d([-5.0, 5.0])
    ax.set_ylim3d([-5.0, 5.0])
    ax.set_zlim3d([-5.0, 0.0])

    def __init__(self, control_queue=None):
        self.control_queue = control_queue
        self.chassis.updateVelocity((.01,.00),(.00))
        initialSegments = self.chassis.step()
        self.lines = [self.ax.plot([dat[0][0],dat[1][0]],[dat[0][1],dat[1][1]],[dat[0][2],dat[1][2]])[0] for dat in initialSegments]

    def update_lines(self, num, lines, chassis):
        if self.control_queue and not self.control_queue.empty():
            control_vector = self.control_queue.get(False)
            self.chassis.updateVelocity(control_vector[0], control_vector[1])
        newSegments = self.chassis.step()
        for line,data in zip(self.lines,newSegments):
            line.set_data([[data[0][0],data[1][0]],[data[0][1],data[1][1]]])
            line.set_3d_properties([data[0][2],data[1][2]])

    def show(self):
        self.line_ani = animation.FuncAnimation(self.fig, self.update_lines, 2000, fargs=(self.lines,self.chassis), interval=1, blit=False)
        plt.show()

if __name__ == '__main__':
    plant = SuperModel()
    plant.show()
