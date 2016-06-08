import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

import cv2
import cv

log_p = np.asarray(cv2.cv.Load("log_p.yml"))

n = log_p.shape[0]

mpl.rcParams['legend.fontsize'] = 10

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(log_p[:,0], log_p[:,1], log_p[:,2], label='Estimated trajectory')
ax.legend()

plt.show()