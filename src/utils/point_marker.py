class PointMarker(object):
  def __init__(self):
    self.x = 0
    self.y = 0
    self.z = 0
    self.mark_size = .1

  def set_command(self, x, y, z): # Commanded point must be in leg's frame
    self.x = x
    self.y = y
    self.z = z

  def update_state(self, time_elapsed):
    pass

  def get_segments(self):
    x = self.x
    y = self.y
    z = self.z
    return [((x-self.mark_size, y, z,1.), (x+self.mark_size,y,z,1.)),
          ((x, y-self.mark_size, z,1.), (x,y+self.mark_size,z,1.)),
          ((x, y, z-self.mark_size,1.), (x,y,z+self.mark_size,1.))]
