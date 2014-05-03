import ctypes
from ctypes import c_double, Structure, cdll
from numpy.ctypeslib import ndpointer

lib = ctypes.cdll.LoadLibrary('./libdemo.so')

class POINT(Structure):
  _fields_ = [('x', c_double), ('y', c_double), ('z', c_double)]

#lib.func.restype = ndpointer(dtype=c_double, shape=(3,))
lib.fk_math.restype = POINT
lib.fk_math_cache.restype = POINT


#p = POINT(1.0, 2.0, 3.0)
#print p.x
#print p.y
#print p.z

#p = lib.fk_math(c_double(1.0), c_double(2.0), c_double(3.0), c_double(.1), c_double(.1), c_double(.1))
#print p.x
#print p.y
#print p.z

def f_c(x, y, z, ca, fa, ta):
  p = lib.fk_math(c_double(x), c_double(y), c_double(z), c_double(ca), c_double(fa), c_double(ta))
  return (p.x, p.y, p.z)

def f_c_cache(x, y, z, ca, fa, ta):
  p = lib.fk_math_cache(c_double(x), c_double(y), c_double(z), c_double(ca), c_double(fa), c_double(ta))
  return (p.x, p.y, p.z)

def trial():
  p = lib.trial()
