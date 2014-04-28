__author__ = 'glalonde'

from math import acos, sqrt, pi, atan2
import numpy as np
from kinematics.utils.transform_math import get_unfinished_dh_mat, finish_dh_mat
from staging.leg_state import LegState

#Does IK assuming point is relative to leg
def IK_3_joint(self, point):
    #Ignore Z value and use only the axial offsets, with the X and Y coords to get the coxa angle
    coxaAngle = np.arctan2(point[1],point[0])+np.arcsin((self.d[1]+self.d[2])/np.sqrt(point[0]**2+point[1]**2))

    #Calculate the inverse coxaDH, and apply to the point to hop into the femur coordinate system
    femurPoint = np.dot(np.linalg.inv(finish_dh_mat(self.coxaDHP, coxaAngle)), np.append(point,[1]))

    #Now we should only have to care about Y and X
    #Only two possibilities from here on
    target = femurPoint[0]**2+femurPoint[1]**2
    #target - range > 0: no solution
    #|target - range| < epsilon: very close/one solution
    #target - range < 0: two solutions
    targetDir = atan2(femurPoint[1], femurPoint[0])

    canReach = 0
    if target > self.range[1]:
        #Ah balls:
        solution1 = (coxaAngle, targetDir, 0)
    elif target < self.range[2]:
        solution1 = (coxaAngle, 0, 0)
    else:
        #is dat sum LAW OF COSINES?!
        #print (self.r[1]**2+self.r[2]**2-target)/(2*self.r[1]*self.r[2])
        thetaA = acos((self.r[1]**2+self.r[2]**2-target)/(2*self.r[1]*self.r[2]))  # The 'inner tibia angle'

        #So yeah, first time I did this, I used law of sines to get the second angle, in so doing,
        #I ignored the rule that every kid is taught in like 5th grade that you can't uniquely determine
        #the properties of a triangle with an angle and two sides where the angle isn't between the sides....
        #So I used law of cosines again.
        thetaB = acos((self.r[1]**2+target-self.r[2]**2)/(2*self.r[1]*sqrt(target)))

        reachStatus = thetaB

        solution1 = LegState(coxaAngle, targetDir + thetaB, thetaA - pi)
        canReach = self.range[2] - target
        #solution2 = (coxaAngle,targetDir - thetaB, pi - thetaA)
    #TODO also return canReach and do something intelligent with it
    return solution1


def get_IK_function(leg):
    if leg.num_joints is 3:
        return IK_3_joint