import math
import leg_model as lm
import pose
from utils.point_marker import PointMarker
import leg_controller as lc
from ik_3dof import IK_3DoF
import time
from utils.interpolators import *

ESTIMATE_REFRESH_RATE=.05

class ChassisController(object):
  def __init__(self, chassis_model):
    self.sim_chassis = chassis_model
    self.current_pose = pose.Pose(0,0,0,0,0,0)
    self.leg_controllers = [lc.LegController(leg, IK_3DoF, legpose, self.current_pose) for leg, legpose in zip(self.sim_chassis.legs, self.sim_chassis.leg_poses)]
    self.step_state = True

    # Movement state
    self.deadline = 0
    self.current_time = 0
    self.interval_estimate = ESTIMATE_REFRESH_RATE
    self.pose_interpolator = None
      
  def set_command(self, cmd):
    if cmd[0] == 'STEP':
      self.step(cmd[1], cmd[2], cmd[3])
    elif cmd[0] == 'POSE':
      self.update_pose_command(cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6], cmd[7])

  def step(self, x, y, deadline):
    if self.step_state:
      x *= -1
      y *= -1
      self.step_state = False
    else:
      self.step_state = True
    for i in [0,2,4]:
      hp = self.leg_controllers[i].home_point
      self.leg_controllers[i].set_command((hp[0] + x, hp[1] + y, hp[2], deadline, self.step_state))

    for i in [1,3,5]:
      hp = self.leg_controllers[i].home_point
      self.leg_controllers[i].set_command((hp[0] - x, hp[1] - y, hp[2], deadline, not self.step_state))

  def update_pose_command(self, dx, dy, dz, dyaw, dpitch, droll, deadline):
    self.deadline = deadline
    self.current_time = 0
    start = self.current_pose.as_tuple()
    end = (start[0] + dx, start[1] + dy, start[2] + dz, start[3] + dyaw, start[4] + dpitch, start[5] + droll)
    self.pose_interpolator = get_linear_interpolator(start, end)

  def update_state(self, time_elapsed):
    self.sim_chassis.update_state(time_elapsed)

    # Update current pose if necessary
    self.current_time += time_elapsed
    self.interval_estimate = .5*self.interval_estimate + .5*time_elapsed
    if self.deadline > 0 and self.pose_interpolator is not None:
      progress = (self.current_time + self.interval_estimate)/self.deadline
      if progress > 1:
        progress = 1
        self.deadline = -1
      np = self.pose_interpolator(progress)
      self.current_pose = pose.Pose.from_tuple(np)
      for leg_controller in self.leg_controllers:
        leg_controller.set_base_pose(self.current_pose)

    # Calculate and send out leg commands
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
