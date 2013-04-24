from chassis import *
from math import pi,sin,cos

def getNextStep(legNum, chassis, desiredChassis):
  leg = chassis[legNum]
  #get intersection between reachable areas of currentBody pose and desiredBody pose
  #This represents the viable regions for steps for which these poses can be achieved
  #Body pose is ([x,y,z],roll,pitch,yaw)
  zStart = chassis.legDHMats[leg][2][3]       #The z position of the hip at the beginning of the step
  zEnd = desiredChassis.legDHMats[leg][2][3]  #The z position of the hip at the end of the step



  #Use these heights to determine the reachable regions
  (innerRadiusStart, outerRadiusStart) = getReachLimits(chassis.legs[legNum], zStart)
  if zStart is not zEnd:
    (innerRadiusEnd, outerRadiusEnd) = getReachLimits(chassis.legs[legNum], zEnd)
  else:
    (innerRadiusEnd, outerRadiusEnd) = (innerRadiusStart, outerRadiusStart)

  #Average all of the limits to get the on furthest from them.
  #This assumes the limits overlap. If they don't, we're hosed.
  targetRadius = (innerRadiusStart + outerRadiusStart + innerRadiusEnd + outerRadiusEnd)/4

  


#Assumes the max femur angle is positive and the min femur angle is negative
#Should probably not rely on this...
def getReachLimits(leg, z):
  zNorm = z**2
  #get min with smallest tibia angle
  minTibiaAngle = max(0, leg.angleRange[2][0]+pi)
  minFootFemurNorm = leg.r[1]**2+leg.r[2]**2-2*leg.r[1]*leg.r[2]*cos(minTibiaAngle)**2
  if minFootFemurNorm < zNorm:
    minRadius = 0
  else:
    minRadius = sqrt(minFootFemurNorm - zNorm)

  maxTibiaAngle = min(pi, leg.angleRange[2][1]+pi)
  maxRadius = sqrt(leg.r[1]**2+leg.r[2]**2-2*leg.r[1]*leg.r[2]*cos(maxTibiaAngle) - zNorm)

  return (minRadius,maxRadius)


