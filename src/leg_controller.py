import leg_model
import time

STEP_HEIGHT = 0
OVERSHOOT_PERCENT = .5
ESTIMATE_REFRESH_RATE=.05
MAX_ANGULAR_VELOCITY = 2.

class LegController(object):
  def __init__(self, leg_model, ik_func):
    self.sim_leg = leg_model
    self.ik_func = ik_func
    self.deadline = 0
    self.current_time = 0
    self.interval_estimate = ESTIMATE_REFRESH_RATE
    self.movement_interpolator = None

  # If ever I have real position feedback, use it here.
  # Until then, use internal dead reckoning model
  def get_current_leg_state(self):
    cx, cy, cz, _ = self.sim_leg.get_end_effector()
    return (cx, cy, cz)
    
  def set_command(self, step_command):
    x, y, z, deadline, bezier = step_command
    self.deadline = deadline
    self.current_time = 0
    cx, cy, cz = self.get_current_leg_state()
    start = (cx, cy, cz)
    end = (x,y,z)
    if bezier:
      p1 = (cx, cy, STEP_HEIGHT)
      p2 = (x, y, STEP_HEIGHT)
      self.move_interpolator = get_cubic_bezier_interpolator(start, p1, p2, end)
    else:
      self.move_interpolator = get_linear_interpolator(start, end)

  def get_joint_commands(self, x, y, z):
    ik_solutions = self.ik_func(self.sim_leg.frame, x, y, z)
    if len(ik_solutions) > 0:
      pairs = zip(self.sim_leg.joint_states, ik_solutions[0])
      completion_times = [abs(joint.current_theta - angle)/MAX_ANGULAR_VELOCITY for joint, angle in pairs]
      max_time = max(max(completion_times), .00000001)
      joint_velocities = [MAX_ANGULAR_VELOCITY*time/max_time for time in completion_times]
      return zip(ik_solutions[0], joint_velocities)
    else:
      return None

  def update_state(self, time_elapsed):
    self.sim_leg.update_state(time_elapsed)
    self.current_time += time_elapsed
    self.interval_estimate = .5*self.interval_estimate + .5*time_elapsed
    if self.deadline > 0 and self.move_interpolator is not None:
      progress = (self.current_time + self.interval_estimate)/self.deadline
      if progress > 1:
        progress = 1
        self.deadline = -1
      (x, y, z) = self.move_interpolator(progress)
      joint_commands = self.get_joint_commands(x, y, z)
      self.sim_leg.set_joint_commands(joint_commands)
      return joint_commands
    return None


UPDATE_INTERVAL = .02
# Input: tuples with (x, y, z, linear?)
# Output: [(angle, velocity), ...]
def leg_controller_updater(controller, step_command_input, servo_command_output):
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
    
  

# Parametric cubic bezier equation from p0 to p3
# Initially traveling towards p1 from p0
# Ends traveling to p3 from p2.
def get_cubic_bezier_interpolator(p0, p1, p2, p3):
  return lambda t: (((1-t)**3)*p0[0] + ((1-t)**2)*3*t*p1[0] + (t**2)*3*(1-t)*p2[0] + (t**3)*p3[0], 
                    ((1-t)**3)*p0[1] + ((1-t)**2)*3*t*p1[1] + (t**2)*3*(1-t)*p2[1] + (t**3)*p3[1], 
                    ((1-t)**3)*p0[2] + ((1-t)**2)*3*t*p1[2] + (t**2)*3*(1-t)*p2[2] + (t**3)*p3[2]) 

def get_linear_interpolator(p0, p1):
  return lambda t: ((1-t)*p0[0] + t*p1[0], (1-t)*p0[1] + t*p1[1], (1-t)*p0[2] + t*p1[2])

def test():
  pass

if __name__ == '__main__':
  test()
