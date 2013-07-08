from math import pi

class chassisParams():
  #DYNAMIC PARAMETERS(initial setting)
  #Position and orientation of chassis:
  position = [0,0,0]
  yaw = 0
  pitch = 0
  roll = 0
  #Leg Angles
  theta = [[0,      pi/4, -pi/2],
          [ -pi/6,  pi/4, -pi/2],
          [ pi/6,   pi/4, -pi/2],
          [ 0,      pi/4, -pi/2],
          [ -pi/6,  pi/4, -pi/2],
          [ pi/6,   pi/4, -pi/2]]



  #STATIC PARAMETERS(should never change)
  numLegs = 6
  #Leg parameters:
  #alpha is the relative angles of rotation for the joints. Coxa is pi/2 off femur, which is 0 off tibia, which is 0 off foot
  alpha = [[pi/2, 0, 0]]*6

  #Radius is the radius of the joints
  radius = [[.5, 1.5, 2]]*6
  #Displacement is the axial offset, could be the height of a servo horn for example(Be sure to account for left side vs right side)
  displacement = [[0,0,0]]*6
  
  #Angle ranges
  angleRange = [[[-pi/2, pi/2],[-pi/2,pi/2],[-pi,0]]]*6


  #Angles for the legs around the chassis
  legThetas = [i*pi/3 for i in xrange(6)]
  #Angles for the coxa, should probably be 0
  legAlphas = [0]*6
  #distance to coxa from 'center' of chassis
  legRadii = [2,1,2,2,1,2]
  #offset of coxa from center of chassis
  legDisps = [-.5]*6
