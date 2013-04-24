from math import pi
from leg import leg
from pose import pose

class chassisParams():
  #DYNAMIC PARAMETERS(initial setting)
  #Position and orientation of chassis:
  chassisPose = pose((0,0,0),(0,0,0))

  #STATIC PARAMETERS(should never change)
  #Leg parameters:
  #alpha is the relative angles of rotation for the joints. Coxa is pi/2 off femur, which is 0 off tibia, which is 0 off foot
  numLegs = 6
  alpha = [[pi/2, 0, 0]]*6

  #Radius is the radius of the joints
  radius = [[.5, 1.5, 2]]*6

  #Displacement is the axial offset, could be the height of a servo horn for example
  #(Be sure to account for left side vs right side)
  displacement = [[0,0,0]]*6
  
  #Angle ranges
  angleRange = [[[-pi/2, pi/2],[-pi/2,pi/2],[-pi,0]]]*6

  #The actual leg objects are the only params that make it into the chassis
  legs = [leg(alpha[i],radius[i],displacement[i],angleRange[i]) for i in xrange(numLegs)]

  legPose = [ pose((1,1,-.5),(pi/2,0,0)),
              pose((1,0,-.5),(pi/2,0,0)),
              pose((1,-1,-.5),(pi/2,0,0)),
              pose((-1,-1,-.5),(-pi/2,0,0)),
              pose((-1,0,-.5),(-pi/2,0,0)),
              pose((-1,1,-.5),(-pi/2,0,0))]

