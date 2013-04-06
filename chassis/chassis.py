#everything is relative to 'center of mass'

class chassis():
  coxaLen = [0]*6
  coxaMaxAngle = [0]*6
  coxaMinAngle = [0]*6
  coxaRadius = [0]*6

  femurLen = [0]*6
  femurMaxAngle = [0]*6
  femurMinAngle = [0]*6
  femurRadius = [0]*6

  tibiaLen = [0]*6
  tibiaMaxAngle = [0]*6
  tibiaMinAngle = [0]*6
  tibiaRadius = [0]*6


  #   0 1
  #  5   2
  #   4 3

  #1 leg specified => radially symmetric
  #2 legs specified => front-back/side symmetric
  #3 legs specified => side symmetric
  #4 legs specified => front-back symmetric
  #5 legs specified => wtf man
  #6 legs specified = no symmetry

  def __init__(robotfile):
    #slurp deets from file


