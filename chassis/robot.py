from chassis import frame


class robot():
  frame
  pose
  stance

  def __init__(self, frame, pose, stance):
    self.frame = frame
    self.pose = pose
    self.stance = stance

  def setDefaultPose(self):
    self.pose = self.frame.getDefaultPose()
