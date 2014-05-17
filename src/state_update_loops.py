import time

UPDATE_INTERVAL = .01


# Args:
# input_pipe_func is a list of (input_pipe, input_func) tuples. Input comes in on input_pipe and is sent to input_func
# After all input has been processed, update_func is called with an argument of the number of seconds that have passed
# output_func_pipe is a list of (output_func, output_pipe) tuples. Output is generated with output_func and sent down output_pipe
# So it looks like: (input_pipe)->(input_func)->(update_func(time_elapsed))->(output_func)->(output_pipe)

def generic_update_loop(input_pipe_func, update_func, output_func_pipe):
  last_time = time.time()
  while True:
    start_time = time.time()
    for (input_pipe, input_func) in input_pipe_func:
      input_signal = None
      while input_pipe.poll():
        input_signal = input_pipe.recv()
      if input_signal == 'KILL':
        for (_, output_pipe) in output_func_pipe:
          output_pipe.send('KILL')
        return
      elif input_signal is not None:
        input_func(input_signal)
    current_time = time.time()
    update_func(current_time - last_time)
    last_time = current_time
    for (output_func, output_pipe) in output_func_pipe:
      output_signal = output_func()
      if output_signal is not None:
        output_pipe.send(output_func())
    time_left = UPDATE_INTERVAL - (time.time() - start_time)
    if time_left > 0:
      time.sleep(time_left)

# Chassic Controller update loop
# Input: tuples with (x, y, z, linear?)
# Output: [(angle, velocity), ...]
def chassis_controller_updater(controller, chassis_command_input, servo_command_output, pose_update_output):
  last_time = time.time()
  while True:
    start_time = time.time()
    command = None
    while chassis_command_input.poll():
      command = chassis_command_input.recv()
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
    pose_update_output.send(controller.current_pose.as_tuple())
    time_left = UPDATE_INTERVAL - (time.time() - start_time)
    if time_left > 0:
      time.sleep(time_left)


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
    time_left = UPDATE_INTERVAL - (time.time() - start_time)
    if time_left > 0:
      time.sleep(time_left)


# Leg Controller update loop
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
    time_left = UPDATE_INTERVAL - (time.time() - start_time)
    if time_left > 0:
      time.sleep(time_left)

# Leg Model Update loop
# Input: [(angle, velocity), ...]
# Output: segments
def leg_model_updater(leg_model, servo_command_input, segment_output):
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
      leg_model.set_joint_commands(command)
    current_time = time.time()
    leg_model.update_state(current_time - last_time)
    last_time = current_time
    segment_output.send(leg_model.get_segments())
    time_left = UPDATE_INTERVAL - (time.time() - start_time)
    if time_left > 0:
      time.sleep(time_left)


def pipe_echo(input_pipe):
  while True:
    input = input_pipe.recv()
    if input == 'KILL':
      return
    else:
      print input
