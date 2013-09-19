__author__ = 'glalonde'

import parts_library


parts_library.load()

chassis = parts_library.get_chassis('default-hex')

print chassis.legs

