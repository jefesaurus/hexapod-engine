import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
from robot import robot












mpl.rcParams['legend.fontsize'] = 10


fig = plt.figure()
ax = fig.gca(projection='3d')

ax.plot([0,1],[0,1],[0,1], label='linez')
ax.legend()
plt.show()

