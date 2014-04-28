from math import pi, atan, acos, sqrt, atan2

# Inverse Kinematics for a very specific 3DoF leg structure
def IK_3DoF(kinematic_chain, x, y, z):
  coxa, femur, tibia = kinematic_chain.kinematic_pairs
  x, y, z = float(x), float(y), float(z)

  # The combined offset along the axes of rotation for the femur and tibia
  d = femur.d + tibia.d
  # There is only one possible coxa angle for any point
  det = x**2 + y**2 - d**2
  if det <= 0:
    return [(0, 0, 0)]
  elif d == y: # Some weird degenerate case. Couldn't quite figure out why, but this gets around it
    coxa_angle = 2*atan(y/x)
  else:
    coxa_angle = 2*atan((x - sqrt(x**2 + y**2 - d**2))/(d - y))

  # Now move into the coxa's frame using the calculated angle
  nx, ny, nz, _ = kinematic_chain.inv_transform_funcs[0]((x,y,z), coxa_angle).T[0]

  # Now we only care about X and Y. Z should equal d
  target = nx**2+ny**2
  femur.r = femur.r
  tibia.r = tibia.r
  range = (femur.r + tibia.r)**2  
  target_dir = atan2(ny, nx)
  if target > range:      # Too far
    solutions = [(coxa_angle, target_dir, 0)]
  elif target < (femur.r - tibia.r)**2: # Too close
    solutions = [(coxa_angle, 0, 0)]
  else:
    # law of cosines
    theta_a = acos((femur.r**2 + tibia.r**2 - target)/(2*femur.r * tibia.r))  # The 'inner tibia angle'
    theta_b = acos((femur.r**2 + target - tibia.r**2)/(2*femur.r * sqrt(target)))

    # TODO use servo range limits to select between the two solutions
    solutions = [(coxa_angle, target_dir + theta_b, theta_a - pi), (coxa_angle, target_dir - theta_b, pi - theta_a)]
  return solutions
