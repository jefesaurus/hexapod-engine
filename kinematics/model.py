
from math import *


from chassis import *
from chassisParams import chassisParams
from tripod import *

class animatedChassis():

  #Parameters
  groundZ = -1.5
  velocity = (.05,.00)
  angularVelocity = 0.05
  tripod = ((0,2,4),(1,3,5))
  ripple = ((3,),(0,),(4,),(2,),(5,),(1,))                    
  tripple = ((0,),(4,),(2,),(),(),(),(5,),(1,),(3,),(),(),()) #Time stretched tripod
  wave = ((0,),(1,),(2,),(3,),(4,),(5,))                  #Shitty gait

  maxStepSize = 1 #adjust meeee
  minStepSize = 0 #adjust meeee tooooo
  gait = tripod

  #Local state
  cp = chassisParams()
  c = chassis(cp)
  numLegs = len(c.legs)
  time = 0 

  #Init procedure
  speed = sqrt(velocity[0]**2+velocity[1]**2)
  if speed == 0:
    dir = (0,0)
  else:
    dir = (velocity[0]/speed, velocity[1]/speed)

  numBins = len(gait)
  gaitBinTime = 20   #Initial gaitBinTime determines all free parameters. Does change.
  stepSize = speed*numBins*gaitBinTime
  print stepSize

  #Adjust the stepRate and stepSize if stepSize is out of limits
  if stepSize > maxStepSize:
    stepRate = speed/maxStepSize
    gaitBinTime = ceil(1./(stepRate*numBins))
    stepSize = speed*gaitBinTime*numBins
  elif stepSize < minStepSize:
    stepRate = speed/minStepSize
    gaitBinTime = ceil(1./(stepRate*numBins))
    stepSize = speed*gaitBinTime*numBins

  gaitTime = numBins*gaitBinTime

  targets = [getNextStep(i, c, c.chassisPose, groundZ) for i in xrange(numLegs)]

  def step(self):
    self.time += 1
    self.c.updatePose((self.velocity[0], self.velocity[1], 0),self.angularVelocity, 0, 0)

    currentStep = self.time%self.gaitTime
    if currentStep%self.gaitBinTime == 0:
      currentBin = int(currentStep)/int(self.gaitBinTime)
      legsToMove = self.gait[currentBin]
      oldPose = self.c.chassisPose
      newPose = pose((oldPose.position[0]+self.dir[0]*self.stepSize,oldPose.position[1]+self.dir[1]*self.stepSize,oldPose.position[2]), (oldPose.yaw+self.angularVelocity*self.gaitTime, oldPose.pitch, oldPose.roll))

      for i in legsToMove:
        self.targets[i] = getNextStep(i, self.c, newPose, self.groundZ)


    newThetas = [self.c.getAngles(self.targets[i], i ) for i in xrange(self.numLegs)]
    return self.c.getChassisSegments(newThetas)
   
