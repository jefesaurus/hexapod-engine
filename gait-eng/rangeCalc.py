#This module takes in an orientation and set of physical parameters
#and outputs the range of end effector movement



#returns the realizable range in the specified angle on the specified leg
#2DPoint relative to projection of hip joint into the ground


#get current attitude from the three points of contact
#
def getRange(heightCOM, attitude, leg, angle):
  
