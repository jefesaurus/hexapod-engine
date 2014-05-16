import math
import leg_model as lm
import pose
from utils.point_marker import PointMarker
import leg_controller as lc
from ik_3dof import IK_3DoF
import time


SHOW_COMMANDED_POINT = True
class ChassisController(object):
  def __init__(self, chassis_model):
    self.sim_chassis = chassis_model
    self.leg_controllers = [lc.LegController(leg, IK_3DoF) for leg in self.sim_chassis.legs]
      
  def set_command(self, chassis_control):
    #(x, y, z, deadline, bezier) = chassis_control
    for leg_controller in self.leg_controllers:
      leg_controller.set_command(chassis_control)

  def update_state(self, time_elapsed):
    self.sim_chassis.update_state(time_elapsed)
    leg_commands = []
    for i, leg_controller in enumerate(self.leg_controllers):
      joint_commands = leg_controller.update_state(time_elapsed)
      if joint_commands is not None:
        leg_commands.append((i, joint_commands))
    self.sim_chassis.set_command(leg_commands)
    return leg_commands
        

UPDATE_INTERVAL = .02
# Input: tuples with (x, y, z, linear?)
# Output: [(angle, velocity), ...]
def chassis_controller_updater(controller, step_command_input, servo_command_output):
  last_time = time.time()
  while True:
    start_time = time.time()
    command = None
    while step_command_input.poll():
      command = step_command_input.recv()
      if command == 'KILL':
        servo_command_output.send('KILL')
        return
    if command is not None:
      controller.set_command(command)
    current_time = time.time()
    servo_commands = controller.update_state(current_time - last_time)
    last_time = current_time
    if servo_commands is not None:
      servo_command_output.send(servo_commands)
    time.sleep(UPDATE_INTERVAL - (time.time() - start_time))



if __name__ == '__main__':
  pass