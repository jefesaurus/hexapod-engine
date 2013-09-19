__author__ = 'glalonde'


import xml.etree.ElementTree as ET
import servo


tree = ET.parse('configurations/servos.xml')
root = tree.getroot()
servos = []
for child in root:
    new_servo = servo.servo()
    servos.append(new_servo.load_from_xml(child))
    print new_servo.angular_speed

print servos