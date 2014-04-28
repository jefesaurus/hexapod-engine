__author__ = 'glalonde'

from math import pi, cos, sin
from utils.getch import _Getch
from commands import commands


class controller:

    def __init__(self, control_queue):
        self.control_queue = control_queue

    def start(self):
        while True:
            control_signal = self._get_control_signal()
            command = self._process_input(control_signal)
            self.control_queue.put(command)

    # Get actual input from the keyboard
    def _get_control_signal(self):
        print 'Press WASD or q'
        next_char = _Getch()
        while True:
            k = next_char()
            if k in 'qwasdop':
                return k

    # Turn the input into a command object with correct data
    def _process_input(control_signal):

        if control_signal == 'q':
            return commands.Quit()

        elif control_signal in 'wasdop':
            speed = .05
            angular_velocity_coeff = .03
            angle = 0

            if control_signal == 'w':
                turn_direction = 0
                angle = 3*pi/2
            elif control_signal == 'd':
                angle = 0
            elif control_signal == 's':
                angle = pi/2
            elif control_signal == 'a':
                angle = pi
            elif control_signal == 'o':
                speed = .001
                turn_direction = 1
            elif control_signal == 'p':
                speed = .001
                turn_direction = -1

            control_vector = ((speed*cos(angle), speed*sin(angle)), angular_velocity_coeff*turn_direction)
            return commands.Walk(control_vector)