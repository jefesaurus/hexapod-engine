def get_linear_interpolator(start, end):
  return lambda t: [(1-t)*start[d_i] + t*end[d_i] for d_i in xrange(len(start))]

def get_cubic_bezier_interpolator(p0, p1, p2, p3):
  return lambda t: [((1-t)**3)*p0[d_i] + ((1-t)**2)*3*t*p1[d_i] + (t**2)*3*(1-t)*p2[d_i] + (t**3)*p3[d_i] for d_i in xrange(len(p0))]
