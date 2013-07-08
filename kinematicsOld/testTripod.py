from chassis import *
from chassisParams import chassisParams
from tripod import *
from pose import pose
from math import *

cp = chassisParams()
c = chassis(cp)

start = pose((0,0,1.5),(0,0,0))
end = pose((0,0,1.5),(pi/2,0,0))
c.chassisPose = start

print getNextStep(0, c,end, 0)
#print getNextStep(1, c, end, 0)
#print getNextStep(2, c, end, 0)
#print getNextStep(3, c, end, 0)
#print getNextStep(4, c, end, 0)
#print getNextStep(5, c, end, 0)
