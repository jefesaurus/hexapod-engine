
import numpy as np


class leg():
  coxa = np.array([0.]*3)
  femur = np.array([0.]*3)
  tibia = np.array([0.]*3)
  pose = np.array([0.]*3)

  def __init__(self, coxa, femur, tibia, pose):
    self.coxa = coxa
    self.femur = femur
    self.tibia = tibia
    self.pose = pose

  def getTransformedPoints(self):
    

def 
