from math import *
from chassis import *
from chassisParams import chassisParams
from tripod import *
from controller import controller


class animatedChassis():
  updatesPerSecond = 60
  updatePeriod = 1.0/updatesPerSecond
  cont = controller.controller()

  #For keeping track of update times
  lastUpdateTime = 0

  cp = chassisParams()
  c = chassis(cp)

  #Last actual step:
  lastStepPose = c.chassisPose

  stepRate = 5
  stepSize = 1

  #Reset timers and vars for the beginning of the next step
  def initNextStep(self):
  #Feet to pick up
  pass

  



  #Main Update Loop
  def update(self, time):
    if time-self.lastUpdateTime < updatePeriod: return

    #We are updating this time
    self.lastUpdateTime = time
    controlVector = cont.getControl()
    oldPose = self.c.chassisPose

    newPose = 


  #Calculates the position on a cycloid given the start and end positions
  # the progress of the step [0,1] and the height of the cycloid
  def interpolateStep(startPos, endPos, progress, height):
    #X
    xRadius = (startPos[0]-endPos[0])/2
    xMid = startPos[0]+xRadius
    X = xMid - xRadius*cos(progress*pi)

    #Y
    yRadius = (startPos[1]-endPos[1])/2
    yMid = startPos[1]+yRadius
    Y = yMid - yRadius*cos(progress*pi)

    #Z
    Z = (height/2)*(1-cos(progress*2*pi))+(endPos[2]-startPos[2])*progress

    return (X,Y,Z)

#Get intersection angle of Dyn with Stat
#Dyn is based at origin
#Stat is based on positive X axis
#Angle 0 is down the x axis
def getIntersectionAngle(baseDist, statAng, statWidth, dynWidth, statLen, dynLen):
  if (dynWidth + statWidth) > baseDist:
    print "Legs too wide for base positioning"
    return None

  statEnd = (baseDist + statLen*cos(statAng) + statWidth*cos(statAng+pi/2), statLen*sin(statAng) + statWidth*sin(statAng+pi/2))
  dynRadiusEff = sqrt(dynLen**2+dynWidth**2) #Effective radius of dynamic leg

  statVector = np.array([statWidth, cos(statAng-pi/2)*statLen, sin(statAng-pi/2)*statLen])
  dynEnd = (dynRadiusEff*cos(dynAng),dynRadiusEff*sin(dynAng))
  #This uses the dot product as in perceptron algorithm to get distance between and offset line and a point
  #Solve this for dynAng
  dist = -statWidth + statVector[1]*dynRadiusEff*cos(dynAng) + statVector[2]*dynRadiusEff*sin(dynAng)

  



#Get Control Vector from controller parsing thread
#Precalculate angles
#Keep track of time on the maestro
#get time at the end of sending commands to maestro
#get start and end times of calculation sequence
#find total time spent on calculations
#combine these to determine how long to wait before sending new commands
#factor in time to send stuff
#calculate theta diffs, and use to set speed of commands
#

#If possible, use position feedback to control speed of servo command


#Every leg has a state
# (percentagePickedUp, percentagePutDown)

    


#three threads
#First thread constantly reads and updates controller info 
#Second thread constantly reads parsed control vectors, and generates movement data
#Third thread checks for available movement data and sends when available



#Main Loop
#Check if in landing range of last destination
#If yes, keep heading there, and finish step if req'd
#Else, get new control vector
#Perhaps, apply smoothing based on prior destinations
#Store it
#Calculate new step
#Calculate first movement
