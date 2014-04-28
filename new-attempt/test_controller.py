import thread
from math import pi,cos,sin
from multiprocessing import Process, Queue
from utils.getch import _Getch

def point_control(input_queue, controllable, marker):
  delta = .25
  x, y, z = 3.0, 0, 0
  while True:
    control_signal = input_queue.get()
    if control_signal == 'q': return
    elif control_signal == 'w': y += delta
    elif control_signal == 'd': x += delta
    elif control_signal == 's': y -= delta
    elif control_signal == 'a': x -= delta
    elif control_signal == 'o': z += delta
    elif control_signal == 'l': z -= delta

    controllable.set_command(x, y, z)
    marker.set_command(x, y, z)

def keyboard(control_queue):
  getch = _Getch()
  while True:
    print 'Press WASDOL or q'
    next_char = _Getch()
    while True:
      k = next_char()
      if k == 'q':
        control_queue.put(k)
        return
      elif k in 'wasdol': control_queue.put(k)

class PointMarker(object):
  def __init__(self):
    self.x = 0
    self.y = 0
    self.z = 0
    self.mark_size = .1

  def set_command(self, x, y, z): # Commanded point must be in leg's frame
    self.x = x
    self.y = y
    self.z = z

  def update_state(self, time_elapsed):
    pass

  def get_segments(self):
    x = self.x
    y = self.y
    z = self.z
    return [((x-self.mark_size, y, z), (x+self.mark_size,y,z)),
          ((x, y-self.mark_size, z), (x,y+self.mark_size,z)),
          ((x, y, z-self.mark_size), (x,y,z+self.mark_size))]

def single_leg_controller():
  import leg_model as lm
  from model_animator import Canvas

  # Set up leg
  leg = lm.get_test_leg() 
  pm = PointMarker()
  input_queue = Queue()
  output_queue = Queue()
  canvas = Canvas()
  canvas.register_drawable(leg)
  canvas.register_drawable(pm)

  # Start controller
  thread.start_new_thread(keyboard, (input_queue,))
  thread.start_new_thread(point_control, (input_queue, leg, pm))
  canvas.show()

def chassis_controller():
  import chassis as sk
  from model_animator import Canvas

  # Set up leg
  chassis = sk.get_test_chassis() 
  pm = PointMarker()
  input_queue = Queue()
  output_queue = Queue()
  canvas = Canvas()
  canvas.register_drawable(chassis)
  canvas.register_drawable(pm)

  # Start controller
  thread.start_new_thread(keyboard, (input_queue,))
  thread.start_new_thread(point_control, (input_queue, chassis, pm))
  canvas.show()

if __name__ == '__main__':
  chassis_controller()
