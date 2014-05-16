import thread
from math import pi,cos,sin
from multiprocessing import Process, Queue
from utils.getch import _Getch

def point_keyboard(step_command_output):
  delta = .75
  x, y, z = 3.0, 0, 0
  bezier = True

  getch = _Getch()
  while True:
    print 'Press WASDOL or q'
    next_char = _Getch()
    while True:
      k = next_char()
      if k == 'q':
        step_command_output.send('KILL')
        return
      elif k == 'w': y += delta
      elif k == 'd': x += delta
      elif k == 's': y -= delta
      elif k == 'a': x -= delta
      elif k == 'o': z += delta
      elif k == 'l': z -= delta
      elif k == 'i':
        if bezier: bezier = False
        else: bezier = True
      step_command_output.send((x,y,z, 1.0, bezier))

def pipe_echo(input_pipe):
  while True:
    input = input_pipe.recv()
    if input == 'KILL':
      return
    else:
      print input
    

def single_leg_controller():
  import leg_model as lm
  import leg_controller as lc
  import model_animator as ma
  from ik_3dof import IK_3DoF
  from multiprocessing import Pipe

  # Set up leg
  model_leg = lm.get_test_leg() 
  leg_controller = lc.LegController(lm.get_test_leg(), IK_3DoF)

  # Inter-thread/process communication pipes
  step_command_output, step_command_input = Pipe()
  servo_command_output, servo_command_input = Pipe()
  segment_output, segment_input = Pipe()

  # Threads and processes
  thread.start_new_thread(point_keyboard, (step_command_input,))
  leg_controller_process = Process(target=lc.leg_controller_updater, args=(leg_controller, step_command_output, servo_command_input))
  model_leg_process = Process(target=lm.leg_model_updater, args=(model_leg, servo_command_output, segment_input))
  segment_supplier = ma.SegmentSupplier(segment_output)

  leg_controller_process.start()
  model_leg_process.start()

  canvas = ma.Canvas()
  canvas.register_drawable(segment_supplier)
  canvas.show()

  leg_controller_process.join()
  model_leg_process.join()

if __name__ == '__main__':
  single_leg_controller()
  #testt_controller()
