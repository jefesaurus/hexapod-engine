__author__ = 'glalonde'

import numpy as np
from math import sin, cos


def get_unfinished_dh_mat(a, r, d):
    ca = cos(a)
    sa = sin(a)
    return np.array([[1, -ca, sa, r], [1, ca, -sa, r], [0, sa, ca, d], [0, 0, 0, 1]])


def finish_dh_mat(mat, theta):
    ct = cos(theta)
    st = sin(theta)
    return mat*np.array([[ct, st, st, ct], [st, ct, ct, st], [1, 1, 1, 1], [1, 1, 1, 1]])