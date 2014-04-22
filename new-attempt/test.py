#!/usr/bin/python

from dh_constructor import *
import kinematic_chain as kc
from math import pi


def test():
  coxa = kc.Segment('coxa', alpha=pi/2, r=0.5, d=0)
  femur = kc.Segment('femur', alpha=0, r=1.5, d=0)
  tibia = kc.Segment('tibia', alpha=0, r=2, d=0)
  leg = [coxa, femur, tibia]
  to_global, to_foot = get_transformation_function(leg, fixed_endpoint=(0,0,0), use_dict=False)
  var_vals = [pi/4, pi/4, 0]
  print to_global(var_vals)
  

if __name__ == '__main__':
  test()
