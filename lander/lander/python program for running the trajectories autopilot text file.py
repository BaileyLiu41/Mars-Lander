# This program uses the trajectories from the autopilot for lander.cpp and plots the resulting graph

import numpy as np
import matplotlib.pyplot as plt
results = np.loadtxt('autopilot_scenario_1_take3.txt')
plt.figure("trajectory for scenario 1")
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(results[:, 0], results[:, 1], label='altitude (km)')
plt.plot(results[:, 0], results[:, 2], label='velocity normal to origin (m/s)')
plt.legend()
plt.show()

