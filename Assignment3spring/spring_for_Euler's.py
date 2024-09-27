# This program uses the trajectories from Euler's method of spring.cpp and plots the resulting graph

import numpy as np
import matplotlib.pyplot as plt
'''results = np.loadtxt('trajectories.txt')
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(results[:, 0], results[:, 1], label='x (m)')
plt.plot(results[:, 0], results[:, 2], label='v (m/s)')
plt.legend()
plt.show() '''

results = np.loadtxt('trajectories_verlet.txt')
plt.figure(2)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(results[:, 0], results[:, 1], label='x (m)')
plt.plot(results[:, 0], results[:, 2], label='v (m/s)')
plt.legend()
plt.show()