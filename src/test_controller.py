import thread
from math import pi,cos,sin
from multiprocessing import Process, Queue
from utils.getch import _Getch

def point_control(input_queue, controllable):
  delta = .75
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

    controllable.set_command(x, y, z, 1.0)

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

def single_leg_controller():
  import leg_model as lm
  from model_animator import Canvas

  # Set up leg
  leg = lm.get_test_leg() 
  input_queue = Queue()
  output_queue = Queue()
  canvas = Canvas()
  canvas.register_drawable(leg)

  # Start controller
  thread.start_new_thread(keyboard, (input_queue,))
  thread.start_new_thread(point_control, (input_queue, leg))
  canvas.show()

def chassis_controller():
  import chassis as sk
  from model_animator import Canvas

  # Set up leg
  chassis = sk.get_test_chassis() 
  input_queue = Queue()
  output_queue = Queue()
  canvas = Canvas()
  canvas.register_drawable(chassis)

  # Start controller
  thread.start_new_thread(keyboard, (input_queue,))
  thread.start_new_thread(point_control, (input_queue, chassis))
  canvas.show()

if __name__ == '__main__':
  single_leg_controller()
