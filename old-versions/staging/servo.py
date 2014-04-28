

class servo:
    #Conversion constants
    #radian_offset + radian_rate = microseconds
    (radian_offset, radian_rate) = (None, None)

    #pot_offset + pot*pot_rate = radians
    (pot_offset, pot_rate) = (None, None)

    def __init__(self, type, angular_speed, max_angle_radians, min_angle_radians,
                 max_angle_microsec=None, min_angle_microsec=None, max_angle_pot=None, min_angle_pot=None):

        self.type = type
        self.angular_speed = angular_speed  # The rated angular speed in radians/second
        self.max_angle_radians = max_angle_radians  # The mechanical limits in radians
        self.min_angle_radians = min_angle_radians
        self.max_angle_microsec = max_angle_microsec  # The mechanical limits in microseconds
        self.min_angle_microsec = min_angle_microsec
        self.max_angle_pot = max_angle_pot  # The mechanical limits from the internal potentiometer
        self.min_angle_pot = min_angle_pot

        # radian_offset + radian_rate = microseconds
        (self.radian_offset, self.radian_rate) = (None, None)

        # pot_offset + pot*pot_rate = radians
        (self.pot_offset, self.pot_rate) = (None, None)



    @classmethod
    def from_xml_node(cls, node):
        type = node.get("name")
        angular_speed = float(node.find('angular-speed').text)
        max_angle_radians = float(node.find('max-radians').text)
        min_angle_radians = float(node.find('min-radians').text)
        return cls(type, angular_speed, max_angle_radians, min_angle_radians)

    def get_microsec_from_radians(self, radians):
        return self.radian_offset + radians*self.radian_rate

    def get_radians_from_pot(self, pot):
        return self.pot_offset + pot*self.pot_rate

    def get_rate_constants(self):
        if self.max_angle_radians and self.min_angle_radians:
            if self.max_angle_microsec and self.min_angle_microsec:
                (self.radian_offset, self.radian_rate) = calculate_conversion_constants((self.max_angle_radian,
                                                                                         self.min_angle_radian),
                                                                                        (self.max_angle_microsec,
                                                                                         self.min_angle_microsec))
            if self.max_angle_pot and self.min_angle_pot:
                (self.pot_offset, self.pot_rate) = calculate_conversion_constants((self.max_angle_pot,
                                                                                   self.min_angle_pot),
                                                                                  (self.max_angle_radians,
                                                                                   self.min_angle_radians))
    def __repr__(self):
        return "Type: " + self.type


# Takes (unit1(max, min), unit2(max, min))
# returns (offset, rate) for eq: (offset + unit1*rate = unit2)
def calculate_conversion_constants(unit1_range, unit2_range):
    rate = (unit2_range[0] - unit2_range[1]) / (unit1_range[0] - unit1_range[1])
    offset = unit2_range[0] - rate*unit1_range[0]
    return offset, rate



