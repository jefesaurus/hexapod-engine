from chassis import *
from math import pi,sin,cos

def getNextStep(legNum, chassisIn, nextPose, groundZ):
  #get intersection between reachable areas of currentBody pose and desiredBody pose
  #This represents the viable regions for steps for which these poses can be achieved
  #Body pose is ([x,y,z],roll,pitch,yaw)
  legPose = chassisIn.legPose[legNum]
  (xStart,yStart,zStart) = chassisIn.chassisPose.toGlobal(legPose.position)
  (xEnd,yEnd,zEnd) = nextPose.toGlobal(legPose.position)

  (midX,midY) = ((xStart+xEnd)/2,(yStart+yEnd)/2)

  midAngle = legPose.yaw-(chassisIn.chassisPose.yaw+nextPose.yaw)/2
  #Use these heights to determine the reachable regions
  (innerRadiusStart, outerRadiusStart) = getReachLimits(chassisIn.legs[legNum], zStart, groundZ)
  if zStart is not zEnd:
    (innerRadiusEnd, outerRadiusEnd) = getReachLimits(chassisIn.legs[legNum], zEnd, groundZ)
  else:
    (innerRadiusEnd, outerRadiusEnd) = (innerRadiusStart, outerRadiusStart)

  #Average all of the limits to get the on furthest from them.
  #This assumes the limits overlap. If they don't, we're hosed.
  targetRadius = (innerRadiusStart + outerRadiusStart + innerRadiusEnd + outerRadiusEnd)/4

  return (midX+targetRadius*sin(midAngle), midY+targetRadius*cos(midAngle), groundZ)

  
  


#Assumes the max femur angle is positive and the min femur angle is negative
#Should probably not rely on this...
def getReachLimits(leg, pointZ, groundZ):
  z = (pointZ-groundZ)
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


