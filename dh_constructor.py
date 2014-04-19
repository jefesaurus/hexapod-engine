#!/usr/bin/python
import sympy as sp
from sympy.utilities import lambdify
import sympy.core.expr
from math import pi

# Represents a single link in the Denavit Hartenberg representation of a robotic manipulator
class Segment:
  def __init__(self, label, alpha, r, d):
    self.label = label
    self.alpha = alpha
    self.r = r
    self.d = d

# takes a list of Denavit Hartenberg parametrized link segments
# and generates the transformation matrix for each of them
# and generates the complete transformation matrix for the entire
# manipulator
def DHMaker(segments):
  dh_matrices = {}
  inverse_dh_matrices = {}
  theta_labels = [str('theta_'+segment.label) for segment in segments]
  alpha_labels = [str('alpha_'+segment.label) for segment in segments]
  r_labels = [str('r_'+segment.label) for segment in segments]
  d_labels = [str('d_'+segment.label) for segment in segments]
  label_to_value = {}

  for segment, theta_label, alpha_label, r_label, d_label in zip(segments, theta_labels, alpha_labels, r_labels, d_labels):
    label = segment.label
    theta = sp.Symbol(theta_label) # The only real variable. Create a symbol for it
    alpha = sp.Symbol(alpha_label)
    r = sp.Symbol(r_label)
    d = sp.Symbol(d_label)
    # Stash values for subbing later:
    label_to_value[alpha] = segment.alpha
    label_to_value[r] = segment.r
    label_to_value[d] = segment.d

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

    inverse_dh_matrices[label] = inverse_dh_matrix
    dh_matrices[label] = dh_matrix

  # Finally, flatten all the matrices into end-to-end transformation matrices
  compound_dh_matrix = sp.eye(4)
  compound_inverse_dh_matrix = sp.eye(4)
  for segment in segments:
    compound_dh_matrix *= dh_matrices[segment.label]
  for segment in reversed(segments):
    compound_inverse_dh_matrix *= inverse_dh_matrices[segment.label]

  # Substitute known values(Everything except theta)
  compound_dh_matrix = compound_dh_matrix.subs(label_to_value)
  compound_inverse_dh_matrix = compound_inverse_dh_matrix.subs(label_to_value)

  # Chop off terms with small coefficients
  compound_dh_matrix = compound_dh_matrix.applyfunc(coeff_chop)
  compound_inverse_dh_matrix = compound_inverse_dh_matrix.applyfunc(coeff_chop)

  return compound_dh_matrix, compound_inverse_dh_matrix, dh_matrices, inverse_dh_matrices, theta_labels

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
# (x,y,z), (theta0, theta1, theta2...)
# and returns a vector for the transformed coordinates
def get_transformation_function(segments):
  coordinate_labels = ['x','y','z']
  x,y,z = sp.symbols(' '.join(coordinate_labels))
  coordinate_vector = sp.Matrix([x,y,z,1])

  dh_mat, inv_dh_mat, _, _, theta_labels = DHMaker(segments)
  
  # Multiply by the coordinate in the space we are transforming from
  transform_matrix = dh_mat*coordinate_vector
  transform_matrix.row_del(3) #Chop off extra row in result matrix
  inverse_transform_matrix = inv_dh_mat*coordinate_vector
  inverse_transform_matrix.row_del(3)

  # Numerically eval everything finally
  transform_matrix = transform_matrix.evalf(chop=True)
  inverse_transform_matrix = inverse_transform_matrix.evalf(chop=True)

  # One more pass of chopping small stuff
  transform_matrix = transform_matrix.applyfunc(coeff_chop)
  inverse_transform_matrix = inverse_transform_matrix.applyfunc(coeff_chop)

  # Bake into a lambda func
  func = lambdify((coordinate_labels, theta_labels), transform_matrix.T, "numpy")
  inv_func = lambdify((coordinate_labels, theta_labels), inverse_transform_matrix.T, "numpy")
  return func, inv_func


# Z-axis is the last axis of rotation
# X-axis is the common normal between the last two axes of rotation(z-axes)
# Y-axis is constrained by the previous two via right hand rule
def test():
  seg1 = Segment('coxa', pi/2, 0.5, 0)
  seg2 = Segment('femur', 0, 1.5, 0)
  seg3 = Segment('tibia', 0, 2, 0)
  segments = [seg1, seg2, seg3]

  f, inv_f = get_transformation_function(segments)

  import random
  import numpy
  import time
  domain = 10
  count = 500
  start = time.time()
  max_wrong = 0.
  for i in range(count):
    x = random.random()*domain - domain/2.
    y = random.random()*domain - domain/2.
    z = random.random()*domain - domain/2.
    nx,ny,nz = f((x,y,z), (-pi/1.5,pi/3,pi/4)).A[0]
    x1, y1, z1 = inv_f((nx,ny,nz), (-pi/1.5,pi/3,pi/4)).A[0]
    max_wrong = max(max_wrong, abs(x1 - x), abs(y1 - y), abs(z1 -z))
  print "Time for %d iterations: < %s seconds"%(count*2,str(time.time() - start))
  print "Largest error: " + str(max_wrong)

if __name__ == '__main__':
  test()
