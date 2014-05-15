#!/usr/bin/python
import sympy as sp
import numpy
from sympy.utilities import lambdify
from sympy.utilities.lambdify import lambdastr
from sympy.utilities.iterables import flatten
from math import pi

from sympy.utilities.codegen import codegen

# takes a list of Denavit Hartenberg parametrized link segments
# and generates the transformation matrix for each of them
# and generates the complete transformation matrix for the entire
# manipulator
def DHMaker(segments):
  dh_matrices = {}
  inverse_dh_matrices = {}
  required_parameters = []
  for segment in segments:
    if segment.theta is not None:
      theta = segment.theta
    else:
      theta = sp.Symbol(str('theta_' + segment.label))
      required_parameters.append(theta)
    if segment.alpha is not None:
      alpha = segment.alpha
    else:
      alpha = sp.Symbol(str('alpha_' + segment.label))
      required_parameters.append(alpha)
    if segment.r is not None:
      r = segment.r
    else:
      r = sp.Symbol(str('r_' + segment.label))
      required_parameters.append(r)
    if segment.d is not None:
      d = segment.d
    else:
      d = sp.Symbol(str('d_' + segment.label))
      required_parameters.append(d)

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

  actual_reqs = compound_dh_matrix.atoms(sp.Symbol).union(compound_inverse_dh_matrix.atoms(sp.Symbol))
  required_parameters = [str(req) for req in required_parameters if req in actual_reqs]  # This preserves the order in segments
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



# Optional parameters fixed_endpoint and fixed_basepoint
# These are for the cases where you already know the point in the frame you are transforming from or to
# Eg. in FK, the endpoint of the end effector is probably (0,0,0) in its own reference frame or something
# So you can bake that into the equation a priori
# Same thing if you only ever want a single origin transformed for the inverse
def get_sympy_reduction(segments, fixed_endpoint=None, fixed_basepoint=None, coordinate_labels=None, inverse_coordinate_labels=None):
  if fixed_endpoint:
    coordinate_vector = sp.Matrix([fixed_endpoint[0], fixed_endpoint[1], fixed_endpoint[2], 1])
  else:
    x_s, y_s, z_s = sp.symbols(' '.join(coordinate_labels))
    coordinate_vector = sp.Matrix([x_s,y_s,z_s,1])

  if fixed_basepoint:
    inverse_coordinate_vector = sp.Matrix([fixed_basepoint[0], fixed_basepoint[1], fixed_basepoint[2], 1])
  else:
    inv_x_s, inv_y_s, inv_z_s = sp.symbols(' '.join(inverse_coordinate_labels))
    inverse_coordinate_vector = sp.Matrix([inv_x_s, inv_y_s, inv_z_s,1])

  dh_mat, inv_dh_mat, var_names = DHMaker(segments)
  
  # Multiply by the coordinate in the space we are transforming from
  transform_matrix = dh_mat*coordinate_vector
  inverse_transform_matrix = inv_dh_mat*inverse_coordinate_vector

  # Numerically eval everything finally
  transform_matrix = transform_matrix.evalf(chop=True)
  inverse_transform_matrix = inverse_transform_matrix.evalf(chop=True)

  # One more pass of chopping small stuff
  transform_matrix = transform_matrix.applyfunc(coeff_chop)
  inverse_transform_matrix = inverse_transform_matrix.applyfunc(coeff_chop)
  return transform_matrix, inverse_transform_matrix, var_names

# Returns a ready to use lambda function that takes inputs of the form
# transform((x,y,z), {theta_label1: val1, theta_label2: val2...}
# and returns a vector for the transformed coordinates
# LABELS NEED TO MATCH THOSE ON YOUR SEGMENTS. The naming scheme is: PARAMETERNAME_SEGMENTLABEL
# If you need to remember what these are, it also returns a list of what it expects for evaluatation
#
# Optionally, to save some variable manipulation turn off the use_dict flag to supply values directly
# The order of the values is by segment first and within that it is: (theta, alpha, r, d)
def get_transformation_function(segments, fixed_endpoint=None, fixed_basepoint=None, use_dict=True):
  if fixed_endpoint:
    coordinate_labels = []
  else:
    coordinate_labels = ['x','y','z']
  if fixed_basepoint:
    inverse_coordinate_labels = []
  else:
    inverse_coordinate_labels = ['base_x','base_y','base_z']
  trans_mat, inv_trans_mat, var_names = get_sympy_reduction(segments, fixed_endpoint, fixed_basepoint, coordinate_labels, inverse_coordinate_labels)

  # Bake into a lambda func
  base_func = lambdify(flatten((coordinate_labels, var_names)), trans_mat, "numpy")
  base_inv_func = lambdify(flatten((inverse_coordinate_labels, var_names)), inv_trans_mat, "numpy")

  if use_dict:
    if fixed_endpoint:
      func = lambda var_dict: base_func(*flatten([var_dict[var_name] for var_name in var_names])).A
    else:
      func = lambda coords, var_dict: base_func(*flatten((coords, [var_dict[var_name] for var_name in var_names]))).A
    if fixed_basepoint:
      inv_func = lambda var_dict: base_inv_func(*flatten([var_dict[var_name] for var_name in var_names])).A
    else:
      inv_func = lambda coords, var_dict: base_inv_func(*flatten((coords, [var_dict[var_name] for var_name in var_names]))).A
  else:
    if fixed_endpoint:
      func = lambda var_vals: base_func(*flatten(var_vals)).A
    else:
      func = lambda coords, var_vals: base_func(*flatten((coords, var_vals))).A
    if fixed_basepoint:
      inv_func = lambda var_vals: base_inv_func(*flatten(var_vals)).A
    else:
      inv_func = lambda coords, var_vals: base_inv_func(*flatten((coords, var_vals))).A

  return func, inv_func

# generate a C source file at "dest_path" with
# transforms for x,y,z coordinates to and from the end effector
# If intermediate_poitns is true, it generates the transforms to each of the segments
def generate_transformation_code(segments, label, dest_path, fixed_endpoint=None, fixed_basepoint=None, intermediate_points=False):
  if fixed_endpoint:
    coordinate_labels = []
  else:
    coordinate_labels = ['x','y','z']
  if fixed_basepoint:
    inverse_coordinate_labels = []
  else:
    inverse_coordinate_labels = ['base_x','base_y','base_z']
  start_seg = 1
  if not intermediate_points:
    start_seg = len(segments)

  c_funcs = []
  for i in range(start_seg, len(segments)+1):
    trans_mat, inv_trans_mat, var_names = get_sympy_reduction(segments, fixed_endpoint, fixed_basepoint, coordinate_labels, inverse_coordinate_labels)
    inter_label = label + '_' + segments[i-1].label
    c_funcs.extend([('x_to_' + inter_label, trans_mat[0]), ('y_to_' + inter_label, trans_mat[1]), ('z_to_' + inter_label, trans_mat[2]),
  ('x_from_' + inter_label, inv_trans_mat[0]), ('y_from_' + inter_label, inv_trans_mat[1]), ('z_from_' + inter_label, inv_trans_mat[2])])
  codegen(c_funcs, 'C', dest_path, label+'_fk', to_files=True)



# Z-axis is the last axis of rotation
# X-axis is the common normal between the last two axes of rotation(z-axes)
# Y-axis is constrained by the previous two via right hand rule
def test():
  import kinematic_chain as kc
  seg1 = kc.RevoluteJoint('coxa', alpha=pi/2, r=0.5, d=0)
  seg2 = kc.RevoluteJoint('femur', alpha=0, r=1.5, d=0)
  seg3 = kc.RevoluteJoint('tibia', alpha=0, r=2, d=0)
  segments = [seg1, seg2, seg3]

  f, inv_f = get_transformation_function(segments)
              # And here is a dictionary containing values for them
  var_vals = {'theta_coxa':-pi/1.5, 'theta_femur':pi/3, 'theta_tibia':pi/4}


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
    nx,ny,nz,_ = f((x,y,z),var_vals)
    x1, y1, z1,_ = inv_f((nx,ny,nz), var_vals)
    max_wrong = max(max_wrong, abs(x1 - x), abs(y1 - y), abs(z1 - z))
  print "Time for %d iterations: < %s seconds"%(count*2,str(time.time() - start))
  print "Largest error: " + str(max_wrong)

if __name__ == '__main__':
  test()
