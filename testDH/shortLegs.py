 
chassisDH = np.array([[ ct[0], -st[0], 0, ct[0]*r[0]],
[ st[0],  ct[0], 0, r[0]*st[0]],
[   0,    0, 1,     d[0]],
[   0,    0, 0,      1]])
 
 
 
coxaDH = np.array([[ ct[1], -ca[1]*st[1],  sa[1]*st[1], ct[1]*r[1]],
[ st[1],  ca[1]*ct[1], -ct[1]*sa[1], r[1]*st[1]],
[   0,      sa[1],      ca[1],     d[1]],
[   0,        0,        0,      1]])
 
 
femurDH = np.array([[ ct[2], -st[2], 0, ct[2]*r[2]],
[ st[2],  ct[2], 0, r[2]*st[2]],
[   0,    0, 1,     d[2]],
[   0,    0, 0,      1]])
 
 
tibiaDH = np.array([[ ct[3], -st[3], 0,  0],
[ st[3],  ct[3], 0,  0],
[   0,    0, 1, d[3]],
[   0,    0, 0,  1]])
 
 
coxaMatrix = np.array([[ ct[0], -st[0], 0, ct[0]*r[0]],
[ st[0],  ct[0], 0, r[0]*st[0]],
[   0,    0, 1,     d[0]],
[   0,    0, 0,      1]])
 
 
femurMatrix = np.array([[ ct[0]*ct[1] - st[0]*st[1], - ca[1]*ct[0]*st[1] - ca[1]*ct[1]*st[0], ct[0]*sa[1]*st[1] + ct[1]*sa[1]*st[0], ct[0]*r[0] + ct[0]*ct[1]*r[1] - r[1]*st[0]*st[1]]
[ ct[0]*st[1] + ct[1]*st[0], ca[1]*ct[0]*ct[1] - ca[1]*st[0]*st[1], sa[1]*st[0]*st[1] - ct[0]*ct[1]*sa[1], r[0]*st[0] + ct[0]*r[1]*st[1] + ct[1]*r[1]*st[0]],
[ 0, sa[1], ca[1], d[0] + d[1]],
[ 0, 0, 0, 1]])
 
 
 
tibiaMatrix = np.array([[ ct[2]*(ct[0]*ct[1] - st[0]*st[1]) - st[2]*(ca[1]*ct[0]*st[1] + ca[1]*ct[1]*st[0]), - ct[2]*(ca[1]*ct[0]*st[1] + ca[1]*ct[1]*st[0]) - st[2]*(ct[0]*ct[1] - st[0]*st[1]), ct[0]*sa[1]*st[1] + ct[1]*sa[1]*st[0], ct[0]*r[0] + d[2]*(ct[0]*sa[1]*st[1] + ct[1]*sa[1]*st[0]) + ct[2]*r[2]*(ct[0]*ct[1] - st[0]*st[1]) + ct[0]*ct[1]*r[1] - r[1]*st[0]*st[1] - r[2]*st[2]*(ca[1]*ct[0]*st[1] + ca[1]*ct[1]*st[0])],
[ st[2]*(ca[1]*ct[0]*ct[1] - ca[1]*st[0]*st[1]) + ct[2]*(ct[0]*st[1] + ct[1]*st[0]), ct[2]*(ca[1]*ct[0]*ct[1] - ca[1]*st[0]*st[1]) - st[2]*(ct[0]*st[1] + ct[1]*st[0]), sa[1]*st[0]*st[1] - ct[0]*ct[1]*sa[1], r[0]*st[0] - d[2]*(ct[0]*ct[1]*sa[1] - sa[1]*st[0]*st[1]) + ct[2]*r[2]*(ct[0]*st[1] + ct[1]*st[0]) + ct[0]*r[1]*st[1] + ct[1]*r[1]*st[0] + r[2]*st[2]*(ca[1]*ct[0]*ct[1] - ca[1]*st[0]*st[1])],
[ sa[1]*st[2], ct[2]*sa[1], ca[1], d[0] + d[1] + ca[1]*d[2] + r[2]*sa[1]*st[2]],
[ 0, 0, 0, 1]])
 
 
tarsusMatrix = np.array([[ - ct[3]*(st[2]*(ca[1]*ct[0]*st[1] + ca[1]*ct[1]*st[0]) - ct[2]*(ct[0]*ct[1] - st[0]*st[1])) - st[3]*(ct[2]*(ca[1]*ct[0]*st[1] + ca[1]*ct[1]*st[0]) + st[2]*(ct[0]*ct[1] - st[0]*st[1])), st[3]*(st[2]*(ca[1]*ct[0]*st[1] + ca[1]*ct[1]*st[0]) - ct[2]*(ct[0]*ct[1] - st[0]*st[1])) - ct[3]*(ct[2]*(ca[1]*ct[0]*st[1] + ca[1]*ct[1]*st[0]) + st[2]*(ct[0]*ct[1] - st[0]*st[1])), ct[0]*sa[1]*st[1] + ct[1]*sa[1]*st[0], ct[0]*r[0] + d[2]*(ct[0]*sa[1]*st[1] + ct[1]*sa[1]*st[0]) + d[3]*(ct[0]*sa[1]*st[1] + ct[1]*sa[1]*st[0]) + ct[2]*r[2]*(ct[0]*ct[1] - st[0]*st[1]) + ct[0]*ct[1]*r[1] - r[1]*st[0]*st[1] - r[2]*st[2]*(ca[1]*ct[0]*st[1] + ca[1]*ct[1]*st[0])],
[ ct[3]*(st[2]*(ca[1]*ct[0]*ct[1] - ca[1]*st[0]*st[1]) + ct[2]*(ct[0]*st[1] + ct[1]*st[0])) + st[3]*(ct[2]*(ca[1]*ct[0]*ct[1] - ca[1]*st[0]*st[1]) - st[2]*(ct[0]*st[1] + ct[1]*st[0])), ct[3]*(ct[2]*(ca[1]*ct[0]*ct[1] - ca[1]*st[0]*st[1]) - st[2]*(ct[0]*st[1] + ct[1]*st[0])) - st[3]*(st[2]*(ca[1]*ct[0]*ct[1] - ca[1]*st[0]*st[1]) + ct[2]*(ct[0]*st[1] + ct[1]*st[0])), sa[1]*st[0]*st[1] - ct[0]*ct[1]*sa[1], r[0]*st[0] - d[2]*(ct[0]*ct[1]*sa[1] - sa[1]*st[0]*st[1]) - d[3]*(ct[0]*ct[1]*sa[1] - sa[1]*st[0]*st[1]) + ct[2]*r[2]*(ct[0]*st[1] + ct[1]*st[0]) + ct[0]*r[1]*st[1] + ct[1]*r[1]*st[0] + r[2]*st[2]*(ca[1]*ct[0]*ct[1] - ca[1]*st[0]*st[1])],
[ ct[2]*sa[1]*st[3] + ct[3]*sa[1]*st[2], ct[2]*ct[3]*sa[1] - sa[1]*st[2]*st[3], ca[1], d[0] + d[1] + ca[1]*d[2] + ca[1]*d[3] + r[2]*sa[1]*st[2]],
[ 0, 0, 0, 1]])
 
