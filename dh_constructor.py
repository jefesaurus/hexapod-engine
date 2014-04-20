#!/usr/bin/python
import sympy as sp
import numpy
from sympy.utilities import lambdify
from math import pi

# Represents a single link in the Denavit Hartenberg representation of a robotic manipulator
class Segment:
  def __init__(self, label, alpha=None, r=None, d=None, theta=None):
    self.label = label
    self.alpha = alpha
    self.r = r
    self.d = d
    self.theta = theta

# takes a list of Denavit Hartenberg parametrized link segments
# and generates the transformation matrix for each of them
# and generates the complete transformation matrix for the entire
# manipulator
def DHMaker(segments):
  dh_matrices = {}
  inverse_dh_matrices = {}
  for segment in segments:
    if segment.theta is not None:
      theta = segment.theta
    else:
      theta = sp.Symbol(str('theta_' + segment.label)) # Create symbols for each variable
    if segment.alpha is not None:
      alpha = segment.alpha
    else:
      alpha = sp.Symbol(str('alpha_' + segment.label))
    if segment.r is not None:
      r = segment.r
    else:
      r = sp.Symbol(str('r_' + segment.label))
    if segment.d is not None:
      d = segment.d
    else:
      d = sp.Symbol(str('d_' + segment.label))

    # Components of the dh matrix
    rotation_matrix = sp.Matrix([[sp.cos(theta), -sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha)],
             [sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha)],
             [0, sp.sin(alpha), sp.cos(alpha)]])
    inverse_rotation = rotation_matrix.T

    translation_matrix = sp.Matrix([r*sp.cos(theta),r*sp.sin(theta),d])
    inverse_translation = -inverse_rotation*translation_matrix

    last_row = sp.Matrix([[0, 0, 0, 1]])

    # Compose the forwards and inverse DH Matrices
    dh_matrix = sp.Matrix.vstack(sp.Matrix.hstack(rotation_matrix, translation_matrix), last_row)
    inverse_dh_matrix = sp.Matrix.vstack(sp.Matrix.hstack(inverse_rotation, inverse_translation), last_row)

    inverse_dh_matrices[segment.label] = inverse_dh_matrix
    dh_matrices[segment.label] = dh_matrix

  # Finally, flatten all the matrices into end-to-end transformation matrices
  compound_dh_matrix = sp.eye(4)
  compound_inverse_dh_matrix = sp.eye(4)
  for segment in segments:
    compound_dh_matrix *= dh_matrices[segment.label]
  for segment in reversed(segments):
    compound_inverse_dh_matrix *= inverse_dh_matrices[segment.label]

  # Chop off terms with small coefficients
  compound_dh_matrix = compound_dh_matrix.applyfunc(coeff_chop)
  compound_inverse_dh_matrix = compound_inverse_dh_matrix.applyfunc(coeff_chop)

  required_parameters = list(compound_dh_matrix.atoms(sp.Symbol).union(compound_inverse_dh_matrix.atoms(sp.Symbol)))
  return compound_dh_matrix, compound_inverse_dh_matrix, required_parameters

# This recursively traverses the input expression
# Grabs the numerical coefficient of terms and replaces with 0
# if they are smaller than the 'tol'
# Probably not mathematically sound in some cases, so try
# to simplify trig identities and stuff first
def coeff_chop(expr, tol=1e-15):
  if abs(expr.as_coeff_Mul(rational=False)[0]) < tol:
    return 0.
  elif len(expr.args) < 1:
    return expr
  args = []
  for arg in expr.args:
    args.append(coeff_chop(arg, tol))
  return expr.func(*args)


# Returns a ready to use lambda function that takes inputs of the form
# transform((x,y,z), {theta_label1: val1, theta_label2: val2...}
# and returns a vector for the transformed coordinates
# LABELS NEED TO MATCH THOSE ON YOUR SEGMENTS. The naming scheme is: PARAMETERNAME_SEGMENTLABEL
# If you need to remember what these are, it also returns a list of what it expects for evaluatation
#
# Optional parameters fixed_endpoint and fixed_basepoint
# These are for the cases where you already know the point in the frame you are transforming from or to
# Eg. in FK, the endpoint of the end effector is probably (0,0,0) in its own reference frame or something
# So you can bake that into the equation a priori
# Same thing if you only ever want a single origin transformed for the inverse
def get_transformation_function(segments, fixed_endpoint=None, fixed_basepoint=None):
  if fixed_endpoint is None:
    coordinate_labels = ['x','y','z'] # It is assumed that the inverse will not need any fixed coordinates
    x_s, y_s, z_s = sp.symbols(' '.join(coordinate_labels))
    coordinate_vector = sp.Matrix([x_s,y_s,z_s,1])
  else:
    coordinate_vector = sp.Matrix([fixed_endpoint[0], fixed_endpoint[1], fixed_endpoint[2], 1])

  if fixed_basepoint is None:
    inverse_coordinate_labels = ['inv_x','inv_y','inv_z'] # It is assumed that the inverse will not need any fixed coordinates
    inv_x_s, inv_y_s, inv_z_s = sp.symbols(' '.join(inverse_coordinate_labels))
    inverse_coordinate_vector = sp.Matrix([inv_x_s, inv_y_s, inv_z_s,1])
  else:
    inverse_coordinate_vector = sp.Matrix([fixed_basepoint[0], fixed_basepoint[1], fixed_basepoint[2], 1])

  dh_mat, inv_dh_mat, var_names = DHMaker(segments)
  var_names = sorted([str(var) for var in var_names]) # Sort for determinism
  
  # Multiply by the coordinate in the space we are transforming from
  transform_matrix = dh_mat*coordinate_vector
  transform_matrix.row_del(3) #Chop off extra row in result matrix
  inverse_transform_matrix = inv_dh_mat*inverse_coordinate_vector
  inverse_transform_matrix.row_del(3)

  # Numerically eval everything finally
  transform_matrix = transform_matrix.evalf(chop=True)
  inverse_transform_matrix = inverse_transform_matrix.evalf(chop=True)

  # One more pass of chopping small stuff
  transform_matrix = transform_matrix.applyfunc(coeff_chop)
  inverse_transform_matrix = inverse_transform_matrix.applyfunc(coeff_chop)
  # Bake into a lambda func
  if fixed_endpoint is None:
    base_func = lambdify((coordinate_labels, var_names), transform_matrix.T, "numpy")
    func = lambda coords, var_dict: base_func(coords, [var_dict[var_name] for var_name in var_names]).A[0]
  else:
    base_func = lambdify([var_names], transform_matrix.T, "numpy")
    func = lambda var_dict: base_func([var_dict[var_name] for var_name in var_names]).A[0]

  if fixed_basepoint is None:
    base_inv_func = lambdify((inverse_coordinate_labels, var_names), inverse_transform_matrix.T, "numpy")
    inv_func = lambda coords, var_dict: base_inv_func(coords, [var_dict[var_name] for var_name in var_names]).A[0]
  else:
    base_inv_func = lambdify([var_names], inverse_transform_matrix.T, "numpy")
    inv_func = lambda var_dict: base_inv_func([var_dict[var_name] for var_name in var_names]).A[0]

  return func, inv_func, var_names


# Z-axis is the last axis of rotation
# X-axis is the common normal between the last two axes of rotation(z-axes)
# Y-axis is constrained by the previous two via right hand rule
def test():
  seg1 = Segment('coxa', alpha=pi/2, r=0.5, d=5.6)
  seg2 = Segment('femur', alpha=0, r=1.5, d=2.4)
  seg3 = Segment('tibia', alpha=0, r=2, d=1.3)
  segments = [seg1, seg2, seg3]

  f, inv_f, vars = get_transformation_function(segments)
  print vars  # These are the missing parameters from the above definitions
              # And here is a dictionary containing values for them
  var_vals = {'theta_coxa':-pi/1.5, 'theta_femur':pi/3, 'theta_tibia':pi/4}

  # Now you can actually do a transformation super easily like so:
  nx,ny,nz = f((1.,2.,3.),var_vals)

  import random
  import time
  domain = 10
  count = 500
  start = time.time()
  max_wrong = 0.
  for i in range(count):
    x = random.random()*domain - domain/2.
    y = random.random()*domain - domain/2.
    z = random.random()*domain - domain/2.
    nx,ny,nz = f((x,y,z),var_vals)
    x1, y1, z1 = inv_f((nx,ny,nz), var_vals)
    max_wrong = max(max_wrong, abs(x1 - x), abs(y1 - y), abs(z1 - z))
  print "Time for %d iterations: < %s seconds"%(count*2,str(time.time() - start))
  print "Largest error: " + str(max_wrong)

if __name__ == '__main__':
  test()
