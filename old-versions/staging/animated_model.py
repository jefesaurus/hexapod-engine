from math import *

from chassis import *
from chassisParams import chassisParams
from tripod import *

class animated_model():

    #Parameters
    groundZ = -2.5
    tripod = ((0,2,4),(1,3,5))
    ripple = ((3,),(0,),(4,),(2,),(5,),(1,))
    tripple = ((0,),(4,),(2,),(),(),(),(5,),(1,),(3,),(),(),()) #Time stretched tripod
    wave = ((0,),(1,),(2,),(3,),(4,),(5,))                  #Shitty gait

    maxStepSize = 2. #adjust meeee
    minStepSize = .001 #adjust meeee tooooo
    gait = tripod

    #Local state
    cp = chassisParams()
    c = chassis(cp)
    numLegs = len(c.legs)
    targets = [getNextStep(i, c, c.chassisPose, groundZ) for i in xrange(numLegs)]

    gaitStep = 0
    stepProgress = 0

    def step(self):
        self.c.updatePose((self.velocity[0], self.velocity[1], 0),self.angularVelocity, 0, 0)

        if self.stepProgress > 1:
            legsToMove = self.gait[self.gaitStep]
            oldPose = self.c.chassisPose
            newPose = pose((oldPose.position[0]+self.dir[0]*self.stepSize,oldPose.position[1]+self.dir[1]*self.stepSize,oldPose.position[2]), (oldPose.yaw+self.angularVelocity*self.gaitTime, oldPose.pitch, oldPose.roll))

            for i in legsToMove:
                self.targets[i] = getNextStep(i, self.c, newPose, self.groundZ)

            self.gaitStep += 1
            self.stepProgress = 0
            self.gaitStep %= self.numBins

        else:
            self.stepProgress += 1./self.gaitBinTime

        newThetas = [self.c.getAngles(self.targets[i], i ) for i in xrange(self.numLegs)]
        return self.c.getChassisSegments(newThetas)

    def updateVelocity(self, velocity, angularVelocity):
        self.velocity = velocity
        self.angularVelocity = angularVelocity
        self.speed = sqrt(velocity[0]**2+velocity[1]**2)
        if self.speed == 0:
            self.dir = (0,0)
        else:
            self.dir = (velocity[0]/self.speed, velocity[1]/self.speed)

        self.numBins = len(self.gait)
        self.gaitBinTime = 20/self.numBins   #Initial gaitBinTime determines all free parameters. Does change.
        self.stepSize = self.speed*self.numBins*self.gaitBinTime

        #Adjust the stepRate and stepSize if stepSize is out of limits
        if self.stepSize > self.maxStepSize:
            stepRate = self.speed/self.maxStepSize
            self.gaitBinTime = max(1,floor(1./(stepRate*self.numBins)))
            self.stepSize = self.speed*self.gaitBinTime*self.numBins

        elif self.stepSize < self.minStepSize:
            stepRate = self.speed/self.minStepSize
            self.gaitBinTime = max(1,floor(1./(stepRate*self.numBins)))
            self.stepSize = self.speed*self.gaitBinTime*self.numBins

        self.gaitTime = self.numBins*self.gaitBinTime
