


#Control Types:
#Velocity vectors: ("velocity",x,y,z,rotation)
#Single Leg: ("single-leg",x,y,z)
#Body Rotation: ("rotation",x,y,z)

class controller():
  def __init__(self):
    self.type = "velocity"
    self.vector = (1,0,0,0)
  def getControl(self):
    return (self.type, self.vector)
