__author__ = 'glalonde'

class Gait:
    def __init__(self, num_steps, action_list):
        self.num_steps = num_steps
        self.action_list = action_list
        # Action_list is a list of lists of (pick up, put down) tuples for each leg
        self.num_legs = len(action_list)
        self.action_dict = [{} for _ in xrange(self.num_legs)]

    @classmethod
    def from_xml_node(cls, node):
        type = node.get("name")
        angular_speed = float(node.find('angular-speed').text)
        max_angle_radians = float(node.find('max-radians').text)
        min_angle_radians = float(node.find('min-radians').text)
        return cls(type, angular_speed, max_angle_radians, min_angle_radians)


    def generate_actions(self):
        for (leg_index, leg_list) in enumerate(self.action_list):
            last_action = -1
            for (pick_up, put_down) in leg_list:
                if pick_up > put_down:
                    if not put_down > last_action:
                        raise ValueError('Gait is jumbled')
                    for i in xrange(put_down, pick_up):
                        self.action_dict[leg_index][i] = (True, pick_up)
                    last_action = pick_up

                else:
                    if not pick_up > last_action:
                        raise ValueError('Gait is jumbled')
                    for i in xrange(pick_up, put_down):
                        self.action_dict[leg_index][i] = (False, put_down)
                    last_action = put_down

    def get_next_action(self, step, leg_index):
        return self.action_dict[leg_index][step]