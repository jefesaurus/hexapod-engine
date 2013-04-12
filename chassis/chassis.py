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
      f = open(robotFile, 'r')
      for line in f:
        line = line.strip()
        print len(line)
        if (len(line) is 0) | (line[0] is '#'): 
          print "here"
          continue
        arr = np.array([x.strip() for x in line.split([',','|'])])
        #arr.shape = (2,3)
        print arr 

  
      
  

