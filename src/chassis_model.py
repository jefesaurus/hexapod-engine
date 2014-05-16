import math
import leg_model as lm
import pose
from utils.point_marker import PointMarker
import time


SHOW_COMMANDED_POINT = True
class Chassis:
  def __init__(self, legs, leg_poses):
    self.legs = legs
    self.leg_poses = leg_poses
    self.num_legs = len(self.legs)
    self.origin = pose.Pose(0,0,0,0,0,0)
    if self.num_legs > 1:
      leg_points = [(lpose.x, lpose.y, lpose.z, 1.) for lpose in leg_poses]
      self.base_segments = [(leg_points[x-1], leg_points[x]) for x in xrange(self.num_legs)]
    else:
      self.base_segments = []

  def local_to_global(self, point):
    return self.origin.from_frame_mat.dot(point).tolist()

  def leg_to_global(self, leg_index, point):
    return self.origin.from_frame_mat.dot(self.leg_poses[leg_index].from_frame_mat.dot(point)).tolist()

  def global_to_leg(self, leg_index, point):
    return self.leg_poses[leg_index].to_frame_mat.dot(self.origin.to_frame_mat.dot(point)).tolist()

  def get_segments(self):
    segments = [(self.local_to_global(p1), self.local_to_global(p2)) for (p1, p2) in self.base_segments]
    for i, leg in enumerate(self.legs):
      segments.extend([(self.leg_to_global(i, p1), self.leg_to_global(i, p2)) for (p1, p2) in leg.get_segments()])
    return segments

  def update_state(self, time_elapsed):
    for leg in self.legs:
      leg.update_state(time_elapsed)

  def set_command(self, leg_commands):
    for (leg_index, joint_commands) in leg_commands:
      self.legs[leg_index].set_joint_commands(joint_commands)

UPDATE_INTERVAL = .02
def chassis_model_updater(chassis_model, servo_command_input, segment_output):
  last_time = time.time()
  while True:
    start_time = time.time()
    command = None
    while servo_command_input.poll():
      command = servo_command_input.recv()
      if command == 'KILL':
        segment_output.send('KILL')
        return
    if command is not None:
      chassis_model.set_command(command)
    current_time = time.time()
    chassis_model.update_state(current_time - last_time)
    last_time = current_time
    segment_output.send(chassis_model.get_segments())
    time.sleep(UPDATE_INTERVAL - (time.time() - start_time))


def get_test_chassis():
  import math
  midwidth = 2.
  topwidth = 1.5
  height = 2.5

  top_r_p = pose.Pose(topwidth,height,0,math.atan2(height, topwidth),0,0)
  mid_r_p = pose.Pose(midwidth,0,math.atan2(0,midwidth),0,0,0)
  bot_r_p = pose.Pose(topwidth,-height,0,math.atan2(-height,topwidth),0,0)
  bot_l_p = pose.Pose(-topwidth,-height,0,math.atan2(-height,-topwidth),0,0)
  mid_l_p = pose.Pose(-midwidth,0,0,math.atan2(0,-midwidth),0,0)
  top_l_p = pose.Pose(-topwidth,height,0,math.atan2(height,-topwidth),0,0)

  top_r = lm.get_test_leg()
  mid_r = lm.get_test_leg()
  bot_r = lm.get_test_leg()
  bot_l = lm.get_test_leg()
  mid_l = lm.get_test_leg()
  top_l = lm.get_test_leg()

  legs = [top_r, mid_r, bot_r, bot_l, mid_l, top_l]
  leg_poses = [top_r_p, mid_r_p, bot_r_p, bot_l_p, mid_l_p, top_l_p]
  
  return Chassis(legs, leg_poses)

def test():
  chassis = get_test_chassis()

if __name__ == '__main__':
  test()
