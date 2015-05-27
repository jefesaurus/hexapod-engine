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

    # Shared movement state
    self.current_time = 0
    self.interval_estimate = ESTIMATE_REFRESH_RATE

    # Pose state
    self.pose_deadline = 0
    self.pose_start_time = 0
    self.pose_interpolator = None

    # Step state
    self.step_deadline = 0
    self.step_start_time = 0
    self.current_step = 0

    # Gait: iterable of steps, each step is ((legs to pick up), deadline, progress before next step)
    self.gait = (((0, 2, 4), 1., 1.), ((1, 3, 5), 1., 1.)) 

    # Current heading/rotation
    self.x_vel = 0
    self.y_vel = 0
    self.r_vel = 0
      
  def set_command(self, cmd):
    if cmd[0] == 'POSE':
      self.update_pose_command(cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6], cmd[7])
    elif cmd[0] == 'MOVE':
      self.movement_command(cmd[1], cmd[2], cmd[3])

  def movement_command(self, x, y, rot):
    self.x_vel = x
    self.y_vel = y
    self.r_vel = rot
    if self.step_deadline <= 0:
      self.start_gait()

  def update_pose_command(self, dx, dy, dz, dyaw, dpitch, droll, deadline):
    self.pose_start_time = self.current_time
    self.pose_deadline = deadline
    start = self.current_pose.as_tuple()
    end = (start[0] + dx, start[1] + dy, start[2] + dz, start[3] + dyaw, start[4] + dpitch, start[5] + droll)
    self.pose_interpolator = get_linear_interpolator(start, end)

  def start_gait(self):
    self.next_step()

  def next_step(self):
    self.current_step = (self.current_step + 1) % len(self.gait)
    legs, deadline, next_start = self.gait[self.current_step]
    self.step_deadline = deadline
    self.step_start_time = self.current_time
    next_pose = self.current_pose.add_delta(dx=self.x_vel, dy=self.y_vel, dyaw=self.r_vel)
    self.update_pose_command(self.x_vel, self.y_vel, 0,0,0,0, deadline)
    for i in legs:
      hp = self.leg_controllers[i].home_point
      self.leg_controllers[i].set_command((hp[0] - self.x_vel, hp[1] - self.y_vel, hp[2], deadline, True))

  def update_state(self, time_elapsed):
    self.sim_chassis.update_state(time_elapsed)

    # Update internal time
    self.current_time += time_elapsed
    self.interval_estimate = .5*self.interval_estimate + .5*time_elapsed

    # Update current pose
    if self.pose_deadline > 0 and self.pose_interpolator is not None:
      progress = (self.current_time + self.interval_estimate - self.pose_start_time)/self.step_deadline
      if progress > 1:
        progress = 1
        self.pose_deadline = -1
        self.pose_start_time = -1
      np = self.pose_interpolator(progress)
      self.current_pose = pose.Pose.from_tuple(np)
      for leg_controller in self.leg_controllers:
        leg_controller.set_base_pose(self.current_pose)

    # Update step progress
    if self.step_deadline > 0:
      current_progress = (self.current_time - self.step_start_time)/self.step_deadline
      if current_progress > 1:
        self.step_deadline = -1
        self.step_start_time = -1
        self.next_step()

    # Calculate and send out leg commands
    leg_commands = []
    for i, leg_controller in enumerate(self.leg_controllers):
      joint_commands = leg_controller.update_state(time_elapsed)
      if joint_commands is not None:
        leg_commands.append((i, joint_commands))
    self.sim_chassis.set_command(leg_commands)
    return leg_commands
        


if __name__ == '__main__':
  pass
