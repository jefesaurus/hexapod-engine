__author__ = 'glalonde'


class Command:
    type = 'Abstract'
    data = None

    def __repr__(self):
        return 'Command: ' + str(self.type) + ' Payload: ' + str(self.data)


class Quit(Command):
    def __init__(self):
        self.type = 'quit'
        self.data = None


class Walk(Command):
    def __init__(self, control_vector):
        self.type = 'walk'
        self.data = control_vector


class SwitchGait(Command):
    def __init__(self, new_gait):
        self.type = 'switch-gait'
        self.data = new_gait


class BodyTransform(Command):
    def __init__(self, control_vector):
        self.type = 'body-transform'
        self.data = control_vector


class SingleLegTransform(Command):
    def __init__(self, control_vector):
        self.type = 'single-leg-transform'
        self.data = control_vector


class SwitchMode(Command):
    def __init__(self, new_mode):
        self.type = 'switch-mode'
        self.data = new_mode


class GoHome(Command):
    def __init__(self):
        self.type = 'home'
        self.data = None
