import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
from leg import leg 



leg1 = leg(np.array([0,.5,-.25]),np.array([0,1.5,0]),np.array([0,2,0]))

start = np.array([0,0,0])









mpl.rcParams['legend.fontsize'] = 10


fig = plt.figure()
ax = fig.gca(projection='3d')

ax.plot([0,1],[0,1],[0,1], label='linez')
ax.legend()
plt.show()

