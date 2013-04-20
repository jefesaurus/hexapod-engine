#everything is relative to 'center of mass'
import numpy as np


# The assumptions are that the coxa rotate in a plane parallel to the ground
# and the femur/tibia rotate in parallel planes that are orthogonal to the plane of rotation for the coxa

# Each of the limbs is a vector representing the static form of the limb
# for the coxa, this is a 3D vector, as the pertinent planes are orthogonal
# for the femur and tibia vectors, these are 2D vecotrs, with the remaining information contained in the servo angle


class frame():
  chassis = np.array([[0.]*3]*6) #3D Cartesian vectors from an arbitrary "center" to the point of coxa rotation

  coxa = np.array([[0.]*3]*6)   #3D, relative to leg(not chassis)[x,y,z]: x out of right, y out front, z out of top
  femur = np.array([[0.]*2]*6)  #Relative 2D cartesian vectors. Think of it as limb length and planar offset.
  tibia = np.array([[0.]*2]*6)  #[x,y]: x out of right, y out of front. (so essentially, y = length, x = planar offset) 

  coxaAngles = np.array([[0.]*3]*6)  #Angles for the joints: Default(When servo is centered), Max, Min
  femurAngles = np.array([[0.]*3]*6)
  tibiaAngles = np.array([[0.]*3]*6)

  def __init__(self, chassis, coxa, femur, tibia, coxaAngles, femurAngles, tibiaAngles):
    self.chassis = chassis
    self.coxa = coxa
    self.femur = femur 
    self.tibia = tibia 
    self.coxaAngles = coxaAngles
    self.femurAngles = femurAngles
    self.tibiaAngles = tibiaAngles

  def getDefaultPose():
    return np.array([[coxaAngles[i][0], femurAngles[i][0], tibiaAngles[i][0]] for i xrange(len(chassis))])


def readRobotFile(robotFile):
  f = open(robotFile, 'r')

  legParams = []
  
  #Clean up the input file
  for line in f:
    line = line.strip()
    if (len(line) is 0) or (line[0] is '#'): continue
    legParams.append([[float(param) for param in limbParams.split(',')] for limbParams in line.split('|')])

  
  chassis = np.array([[0.]*3]*6) #3D Cartesian vectors from an arbitrary "center" to the point of coxa rotation

  coxa = np.array([[0.]*3]*6)   #3D, relative to leg(not chassis)[x,y,z]: x out of right, y out front, z out of top
  femur = np.array([[0.]*2]*6)  #Relative 2D cartesian vectors. Think of it as limb length and planar offset.
  tibia = np.array([[0.]*2]*6)  #[x,y]: x out of right, y out of front. (so essentially, y = length, x = planar offset) 

  coxaAngles = np.array([[0.]*3]*6)  #Angles for the joints: Default(When servo is centered), Max, Min
  femurAngles = np.array([[0.]*3]*6)
  tibiaAngles = np.array([[0.]*3]*6)

  numLegDefs = len(legParams)
  #Consider possible ways to parse
  if numLegDefs is 1:
    raise NotImplementedError, "Haven't made 1 leg loading yet"
    
  elif numLegDefs is 2:
    #legs 0 and 1 have already been defined
    for i in xrange(numLegDefs):
      chassis[i] = np.array(legParams[i][0])
      coxa[i] = np.array(legParams[i][1])
      femur[i] = np.array(legParams[i][2])
      tibia[i] = np.array(legParams[i][3])
      coxaAngles[i] = np.array(legParams[i][4])
      femurAngles[i] = np.array(legParams[i][5])
      tibiaAngles[i] = np.array(legParams[i][6])
    
    #Leg 2 is the overX-flip for leg 0
    chassis[2] = flipVectorOverX(chassis[0])
    coxa[2] = coxa[0]
    femur[2] = femur[0] 
    tibia[2] = tibia[0]

    #Leg 3 is the overY-flip of leg 1
    #Leg 4 is the overY-flip of leg 2
    #Leg 5 is the overY-flip of leg 0
    for i in xrange(3):
      chassis[5-i] = flipVectorOverY(chassis[i])
      coxa[5-i] = flipVectorOverY(coxa[i])
      femur[5-i] = flipVectorOverY(femur[i]) 
      tibia[5-i] = flipVectorOverY(tibia[i])
      coxaAngles[5-i] = coxaAngles[i]
      femurAngles[5-i] = femurAngles[i]
      tibiaAngles[5-i] = tibiaAngles[i]

  elif numLegDefs is 3:
    raise NotImplementedError, "Haven't made 3 leg loading yet"
  elif numLegDefs is 4:
    raise NotImplementedError, "Haven't made 4 leg loading yet"
  elif numLegDefs is 6:
    raise NotImplementedError, "Haven't made 6 leg loading yet"
  else:
    print "# of leg lines (" + str(numLegDefs) +") is not valid. Must be 1,2,3,4,6"

  return frame(chassis, coxa, femur, tibia, coxaAngles, femurAngles, tibiaAngles)

def flipVectorOverX(vector):
  if len(vector) is 3:
    return [vector[0],-vector[1],vector[2]]
  elif len(vector) is 2:
    return [vector[0],-vector[1]]
def flipVectorOverY(vector):
  if len(vector) is 3:
    return [-vector[0],vector[1],vector[2]]
  elif len(vector) is 2:
    return [-vector[0],vector[1]]
