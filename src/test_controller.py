import thread
import time
import threading
from math import pi,cos,sin
from multiprocessing import Process, Queue
from utils.getch import _Getch

from state_update_loops import *

def point_keyboard(chassis_command_output, command_lock):
  delta = .5
  x, y, z = 0.0, 0.5, 0
  bezier = True

  getch = _Getch()
  while True:
    print 'Press WASD-OL-I or q'
    next_char = _Getch()
    while True:
      k = next_char()
      command_lock.acquire()
      if k == 'q':
        chassis_command_output.send('KILL')
        return
      elif k == 'w':
        chassis_command_output.send(('POSE', 0, 0, 0, 0,0,-delta/4,.25))
      elif k == 'd':
        chassis_command_output.send(('POSE', 0, 0, 0, 0,delta/4,0,.25))
      elif k == 's':
        chassis_command_output.send(('POSE', 0, 0, 0, 0,0,delta/4,.25))
      elif k == 'a':
        chassis_command_output.send(('POSE', 0, 0, 0, 0,-delta/4,0,.25))
      elif k == 'o':
        chassis_command_output.send(('POSE', 0, 0, delta, 0,0,0,.25))
      elif k == 'l':
        chassis_command_output.send(('POSE', 0, 0, -delta, 0,0,0,.25))
      command_lock.release()

def move_keyboard(chassis_command_output, command_lock):
  delta = .5
  x_vel, y_vel, r_vel = 0.0, 0.0, 0.0

  getch = _Getch()
  while True:
    print 'Press WASD-OL-I or q'
    next_char = _Getch()
    while True:
      k = next_char()
      command_lock.acquire()
      if k == 'q':
        chassis_command_output.send('KILL')
        return
      elif k == 'w':
        y_vel += delta
        chassis_command_output.send(('MOVE', x_vel, y_vel, r_vel))
      elif k == 'd':
        x_vel += delta
        chassis_command_output.send(('MOVE', x_vel, y_vel, r_vel))
      elif k == 's':
        y_vel -= delta
        chassis_command_output.send(('MOVE', x_vel, y_vel, r_vel))
      elif k == 'a':
        x_vel -= delta
        chassis_command_output.send(('MOVE', x_vel, y_vel, r_vel))
      elif k == 'o':
        chassis_command_output.send(('POSE', 0, 0, delta, 0,0,0,.25))
      elif k == 'l':
        chassis_command_output.send(('POSE', 0, 0, -delta, 0,0,0,.25))
      command_lock.release()

def step_sender(chassis_command_output, command_lock):
  x = 0.0
  y = 1.2
  step_interval = .3
  while True:
    command_lock.acquire() 
    chassis_command_output.send(('STEP', x, y, step_interval))
    command_lock.release() 
    time.sleep(step_interval)


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
  chassis_command_output, chassis_command_input = Pipe()
  pose_update_output, pose_update_input = Pipe()
  servo_command_output, servo_command_input = Pipe()
  segment_output, segment_input = Pipe()
  command_lock = threading.Lock()

  # Threads and processes
  thread.start_new_thread(move_keyboard, (chassis_command_input,command_lock))
  thread.start_new_thread(step_sender, (chassis_command_input,command_lock))
  chassis_controller_process = Process(target=chassis_controller_updater, args=(chassis_controller, chassis_command_output, servo_command_input, pose_update_input))
  model_chassis_process = Process(target=chassis_model_updater, args=(model_chassis, servo_command_output, segment_input))
  segment_supplier = ma.SegmentSupplier(segment_output, pose_update_output)

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
