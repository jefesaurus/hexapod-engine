import thread
from math import pi,cos,sin
from multiprocessing import Process, Queue
from utils.getch import _Getch

def point_keyboard(step_command_output):
  delta = .5
  x, y, z = 0.0, 0, 0
  bezier = True

  getch = _Getch()
  while True:
    print 'Press WASD-OL-I or q'
    next_char = _Getch()
    while True:
      k = next_char()
      if k == 'q':
        step_command_output.send('KILL')
        return
      elif k == 'w':
        step_command_output.send(('POSE', 0, 0, 0, 0,0,-delta/4,.25))
      elif k == 'd':
        step_command_output.send(('POSE', 0, 0, 0, 0,delta/4,0,.25))
      elif k == 's':
        step_command_output.send(('POSE', 0, 0, 0, 0,0,delta/4,.25))
      elif k == 'a':
        step_command_output.send(('POSE', 0, 0, 0, 0,-delta/4,0,.25))
      elif k == 'o':
        step_command_output.send(('POSE', 0, 0, delta, 0,0,0,.25))
      elif k == 'l':
        step_command_output.send(('POSE', 0, 0, -delta, 0,0,0,.25))
      elif k == 'r':
        step_command_output.send(('STEP', x, y, .25))
      #step_command_output.send((x,y,z, 1.0, bezier))


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

def chassis_controller():
  import chassis_model as cm
  import chassis_controller as cc
  import model_animator as ma
  from multiprocessing import Pipe

  # Set up leg
  model_chassis = cm.get_test_chassis() 
  chassis_controller = cc.ChassisController(cm.get_test_chassis())

  # Inter-thread/process communication pipes
  step_command_output, step_command_input = Pipe()
  servo_command_output, servo_command_input = Pipe()
  segment_output, segment_input = Pipe()

  # Threads and processes
  thread.start_new_thread(point_keyboard, (step_command_input,))
  chassis_controller_process = Process(target=cc.chassis_controller_updater, args=(chassis_controller, step_command_output, servo_command_input))
  model_chassis_process = Process(target=cm.chassis_model_updater, args=(model_chassis, servo_command_output, segment_input))
  segment_supplier = ma.SegmentSupplier(segment_output)

  chassis_controller_process.start()
  model_chassis_process.start()

  canvas = ma.Canvas()
  canvas.register_drawable(segment_supplier)
  canvas.show()

  chassis_controller_process.join()
  model_chassis_process.join()

if __name__ == '__main__':
  chassis_controller()
  #testt_controller()
