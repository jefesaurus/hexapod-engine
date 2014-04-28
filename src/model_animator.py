import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

class Canvas:
  def __init__(self):
    self._init_graphics()
    self.interval = 1
    self.segment_funcs = []
    self.update_funcs = []
    self.last_time = time.time()

  def register_drawable(self, drawable):
    segment_func = drawable.get_segments
    segments = [self.ax.plot([dat[0][0], dat[1][0]], [dat[0][1], dat[1][1]], [dat[0][2], dat[1][2]])[0] for dat in segment_func()]
    self.segment_funcs.append((segment_func, segments))
    self.update_funcs.append(drawable.update_state)

  def _init_graphics(self):
    self.fig = plt.figure()
    self.ax = p3.Axes3D(self.fig)
    self.ax.set_xlim3d([-5.0, 5.0])
    self.ax.set_ylim3d([-5.0, 5.0])
    self.ax.set_zlim3d([-5.0, 0.0])

  def update(self, num):
    current_time = time.time()
    elapsed_time = current_time - self.last_time
    self.update_state(elapsed_time)
    self.last_time = current_time
    self.update_lines()

  def update_state(self, elapsed_time):
    for update_func in self.update_funcs:
      update_func(elapsed_time)

  def update_lines(self):
    for segment_func, old_segments in self.segment_funcs:
      new_segments = segment_func()
      for segment_container, data in zip(old_segments, new_segments):
        segment_container.set_data([[data[0][0], data[1][0]], [data[0][1], data[1][1]]])
        segment_container.set_3d_properties([data[0][2], data[1][2]])

  def show(self):
    self.line_ani = animation.FuncAnimation(self.fig, self.update, 2000, interval=self.interval, blit=False)
    plt.show()

def leg_test():
  import leg_model as lm
  from math import pi

  leg = lm.get_test_leg() 
  leg.set_angles([pi/5, pi/5, -pi/2])
  canvas = Canvas()
  canvas.register_drawable(leg)
  canvas.show()

def chassis_test():
  import chassis as sk
  chassis = sk.get_test_chassis()
  canvas = Canvas()
  canvas.register_drawable(chassis)
  canvas.show()


if __name__ == '__main__':
  chassis_test()
