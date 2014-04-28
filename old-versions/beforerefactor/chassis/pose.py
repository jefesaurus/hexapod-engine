import numpy as np

class stance():
  servoAngles = np.array([[0.]*3]*6])

  def __init__(self, angles):
    self.servoAngles = angles

