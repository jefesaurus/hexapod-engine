import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from datetime import datetime

from parts_library import chassis_library
from dynamic_model import *


class SuperModel:
    def __init__(self, chassis_name):
        self._init_graphics()
        self.interval = 4
        self.old_time = datetime.now()
        self.model = dynamic_model(chassis_library.get_chassis(chassis_name))
        self.lines = []
        initial_segments = self.model.get_chassis_segments()
        self.lines = [self.ax.plot([dat[0][0], dat[1][0]], [dat[0][1], dat[1][1]], [dat[0][2], dat[1][2]])[0]
                      for dat in initial_segments]

    def _init_graphics(self):
        self.fig = plt.figure()
        self.ax = p3.Axes3D(self.fig)
        self.ax.set_xlim3d([-5.0, 5.0])
        self.ax.set_ylim3d([-5.0, 5.0])
        self.ax.set_zlim3d([-5.0, 0.0])

    def update(self, num):
        self.update_model_state(num)
        self.update_lines()

    def update_model_state(self, num):
        #if self.control_queue and not self.control_queue.empty():
        #self.model.set_current_command(self.control_queue.get(False))
        if num % 15 == 0:
            self.model.set_current_command(ChassisState.get_random(self.model.chassis.num_legs))

        new_time = datetime.now()
        self.model.update_servos((new_time - self.old_time).microseconds/1e6)
        self.old_time = new_time

    def update_lines(self):
        new_segments = self.model.get_chassis_segments()
        for line, data in zip(self.lines, new_segments):
            line.set_data([[data[0][0], data[1][0]], [data[0][1], data[1][1]]])
            line.set_3d_properties([data[0][2], data[1][2]])

    def show(self):
        self.line_ani = animation.FuncAnimation(self.fig, self.update, 2000, interval=self.interval, blit=False)
        plt.show()

if __name__ == '__main__':
    plant = SuperModel('default-hex')
    plant.show()
