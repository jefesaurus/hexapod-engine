import numpy as np
from math import pi

#actual angle of the joints
t0, t1, t2, t3 = (0, pi/2, 0, 0)
ct0, ct1, ct2, ct3 = cos(t0), cos(t1), cos(t2), cos(t3)
st0, st1, st2, st3 = sin(t0), sin(t1), sin(t2), sin(t3)

#z axis angle
a0, a1, a2, a3, a4 = (0, pi/2, 0, 0)
ca1 = cos(a1)
sa1 = sin(a1)

#Radius
r0, r1, r2, r3 = (1.5, 1, 1.5, 2)

#normal offset
d0, d1, d2, d3 = (0, 0, 0, 0)

chassisDH = np.array([[ ct0, -st0, 0, ct0*r0],
                      [ st0,  ct0, 0, r0*st0],
                      [   0,    0, 1,     d0],
                      [   0,    0, 0,      1]])
 
 
coxaDH = np.array([[ ct1, -ca1*st1,  sa1*st1, ct1*r1],
                    [ st1,  ca1*ct1, -ct1*sa1, r1*st1],
                    [   0,      sa1,      ca1,     d1],
                    [   0,        0,        0,      1]])
 
 
femurDH = np.array([[ ct2, -st2, 0, ct2*r2],
                    [ st2,  ct2, 0, r2*st2],
                    [   0,    0, 1,     d2],
                    [   0,    0, 0,      1]])
 
 
tibiaDH = np.array([[ ct3, -st3, 0, ct3*r3],
                    [ st3,  ct3, 0, r3*st3],
                    [   0,    0, 1,     d3],
                    [   0,    0, 0,      1]])
 
 
coxa = np.array([[ ct0, -st0, 0, ct0*r0],
                [ st0,  ct0, 0, r0*st0],
                [   0,    0, 1,     d0],
                [   0,    0, 0,      1]])
 
 
femur = np.array([[ ct0*ct1 - st0*st1, - ca1*ct0*st1 - ca1*ct1*st0, ct0*sa1*st1 + ct1*sa1*st0, ct0*r0 + ct0*ct1*r1 - r1*st0*st1],
                  [ ct0*st1 + ct1*st0,   ca1*ct0*ct1 - ca1*st0*st1, sa1*st0*st1 - ct0*ct1*sa1, r0*st0 + ct0*r1*st1 + ct1*r1*st0],
                  [                 0,                         sa1,                       ca1,                          d0 + d1],
                  [                 0,                           0,                         0,                                1]])
 
 
tibia = np.array([[ ct2*(ct0*ct1 - st0*st1) - st2*(ca1*ct0*st1 + ca1*ct1*st0), -ct2*(ca1*ct0*st1 + ca1*ct1*st0) - st2*(ct0*ct1 - st0*st1), ct0*sa1*st1 + ct1*sa1*st0, ct0*r0 + d2*(ct0*sa1*st1 + ct1*sa1*st0) + ct2*r2*(ct0*ct1 - st0*st1) + ct0*ct1*r1 - r1*st0*st1 - r2*st2*(ca1*ct0*st1 + ca1*ct1*st0)],
                  [ st2*(ca1*ct0*ct1 - ca1*st0*st1) + ct2*(ct0*st1 + ct1*st0), ct2*(ca1*ct0*ct1 - ca1*st0*st1) - st2*(ct0*st1 + ct1*st0), sa1*st0*st1 - ct0*ct1*sa1, r0*st0 - d2*(ct0*ct1*sa1 - sa1*st0*st1) + ct2*r2*(ct0*st1 + ct1*st0) + ct0*r1*st1 + ct1*r1*st0 + r2*st2*(ca1*ct0*ct1 - ca1*st0*st1)],
                  [ sa1*st2, ct2*sa1, ca1, d0 + d1 + ca1*d2 + r2*sa1*st2],
                  [ 0, 0, 0, 1]])
 
 
tarsus = np.array([[ - ct3*(st2*(ca1*ct0*st1 + ca1*ct1*st0) - ct2*(ct0*ct1 - st0*st1)) - st3*(ct2*(ca1*ct0*st1 + ca1*ct1*st0) + st2*(ct0*ct1 - st0*st1)), st3*(st2*(ca1*ct0*st1 + ca1*ct1*st0) - ct2*(ct0*ct1 - st0*st1)) - ct3*(ct2*(ca1*ct0*st1 + ca1*ct1*st0) + st2*(ct0*ct1 - st0*st1)), ct0*sa1*st1 + ct1*sa1*st0, ct0*r0 + d2*(ct0*sa1*st1 + ct1*sa1*st0) + d3*(ct0*sa1*st1 + ct1*sa1*st0) + ct2*r2*(ct0*ct1 - st0*st1) - ct3*r3*(st2*(ca1*ct0*st1 + ca1*ct1*st0) - ct2*(ct0*ct1 - st0*st1)) + ct0*ct1*r1 - r3*st3*(ct2*(ca1*ct0*st1 + ca1*ct1*st0) + st2*(ct0*ct1 - st0*st1)) - r1*st0*st1 - r2*st2*(ca1*ct0*st1 + ca1*ct1*st0)],
                  [   ct3*(st2*(ca1*ct0*ct1 - ca1*st0*st1) + ct2*(ct0*st1 + ct1*st0)) + st3*(ct2*(ca1*ct0*ct1 - ca1*st0*st1) - st2*(ct0*st1 + ct1*st0)), ct3*(ct2*(ca1*ct0*ct1 - ca1*st0*st1) - st2*(ct0*st1 + ct1*st0)) - st3*(st2*(ca1*ct0*ct1 - ca1*st0*st1) + ct2*(ct0*st1 + ct1*st0)), sa1*st0*st1 - ct0*ct1*sa1, r0*st0 - d2*(ct0*ct1*sa1 - sa1*st0*st1) - d3*(ct0*ct1*sa1 - sa1*st0*st1) + ct2*r2*(ct0*st1 + ct1*st0) + ct3*r3*(st2*(ca1*ct0*ct1 - ca1*st0*st1) + ct2*(ct0*st1 + ct1*st0)) + r3*st3*(ct2*(ca1*ct0*ct1 - ca1*st0*st1) - st2*(ct0*st1 + ct1*st0)) + ct0*r1*st1 + ct1*r1*st0 + r2*st2*(ca1*ct0*ct1 - ca1*st0*st1)],
                  [ ct2*sa1*st3 + ct3*sa1*st2, ct2*ct3*sa1 - sa1*st2*st3, ca1, d0 + d1 + ca1*d2 + ca1*d3 + r2*sa1*st2 + ct2*r3*sa1*st3 + ct3*r3*sa1*st2],
                  [ 0, 0, 0, 1]])
