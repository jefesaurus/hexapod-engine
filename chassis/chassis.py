#everything is relative to 'center of mass'
import numpy as np
class chassis():
  #   5 0
  #  4   1
  #   3 2

  #1 leg specified => radially symmetric
  #2 legs specified => front-back/side symmetric
  #3 legs specified => side symmetric
  #4 legs specified => front-back symmetric
  #5 legs specified => wtf man
  #6 legs specified = no symmetry



  '''
  The numpy arrays below store the structural data for the robot
  For the first array from center to coxa, this is simply a 3D vector
  For the ones after that, it is a 2D vector (representing cylindrical coordinates from one end of the limb to the other)
  However, these include 3 more pieces of data: Max angle, Min angle, Radius
  The angles are the limits on the rotation for that joint, and the radius is the 'radius' of that limb
  If I feel like implementing some preemptive collision detection
  So the first one looks like [X,Y,Z]X6
  The next 3 look like [X,Y,MaxAngle,MinAngle,Radius]

  '''

  #Chassis shape: Chassis -> Coxa vector is the only 3D one, with Z out of the top of the chassis, X out the front, and Y out of the right
  #All of the other vectors are just 2D, with the angle of the servo making up the third dimension(Cylindrical coordinates)
  centerToCoxaVector = np.array([[0]*3]*6) #Vector from center to pivot point of coxa
  coxaToFemurVector = np.array([[0]*5]*6)   #Vector from terminus of the previous coxa vector to the pivot point for the coxa end of the femur
  femurToTibiaVector = np.array([[0]*5]*6)  #Vector from terminus of the previous femur vector to the pivot point for the femur end of the tibia
  tibiaToEndVector = np.array([[0]*5]*6)    #Vector from terminus of the previous tibia vector to the point of the end effector

  def __init__(self, robotFile=None):
    #slurp deets from file

    if robotFile:
      readRobotFile(robotFile)
        
def readRobotFile(robotFile):
  f = open(robotFile, 'r')
  legParams = []
  
  #Clean up the input file
  for line in f:
    line = line.strip()
    if (len(line) is 0) or (line[0] is '#'): continue
    legParams.append([[float(param) for param in limbParams.split(',')] for limbParams in line.split('|')])
  centerToCoxaVector = np.array([[0.]*3]*6)
  coxaToFemurVector = np.array([[0.]*5]*6)
  femurToTibiaVector = np.array([[0.]*5]*6)
  tibiaToEndVector = np.array([[0.]*5]*6)

  numLegDefs = len(legParams)
  #Consider possible ways to parse
  if numLegDefs is 1:
    #will need some sort of rotate angle procedure. Math is hard
    raise NotImplementedError, "Haven't made 1 leg loading yet"
    
  elif numLegDefs is 2:
    #legs 0 and 1 have already been defined
    for i in xrange(numLegDefs):
      centerToCoxaVector[i] = np.array(legParams[i][0])
      coxaToFemurVector[i] = np.array(legParams[i][1])
      femurToTibiaVector[i] = np.array(legParams[i][2])
      tibiaToEndVector[i] = np.array(legParams[i][3])
    
    #Leg 2 is the overX-flip for leg 0
    centerToCoxaVector[2] = flipVectorOverX(centerToCoxaVector[0])
    coxaToFemurVector[2] = coxaToFemurVector[0]
    femurToTibiaVector[2] = femurToTibiaVector[0] 
    tibiaToEndVector[2] = tibiaToEndVector[0]

    #Leg 3 is the overY-flip of leg 1
    #Leg 4 is the overY-flip of leg 2
    #Leg 5 is the overY-flip of leg 0
    for i in xrange(3):
      centerToCoxaVector[5-i] = flipVectorOverY(centerToCoxaVector[i])
      coxaToFemurVector[5-i] = coxaToFemurVector[i]
      femurToTibiaVector[5-i] = femurToTibiaVector[i] 
      tibiaToEndVector[5-i] = tibiaToEndVector[i]

  elif numLegDefs is 3:
    raise NotImplementedError, "Haven't made 3 leg loading yet"
  elif numLegDefs is 4:
    raise NotImplementedError, "Haven't made 4 leg loading yet"
  elif numLegDefs is 6:
    raise NotImplementedError, "Haven't made 6 leg loading yet"
  else:
    print "# of leg lines (" + str(numLegDefs) +") is not valid. Must be 1,2,3,4,6"

def flipVectorOverX(vector):
  return [vector[0],-vector[1],vector[2]]
def flipVectorOverY(vector):
  return [-vector[0],vector[1],vector[2]] 
