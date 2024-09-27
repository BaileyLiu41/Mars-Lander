# I am assuming that the earth acts as a point mass for simplicity
# Note to self: update the code at some point to include collisions

import numpy as np
import matplotlib.pyplot as plt

# mass, spring constant, initial position and velocity
m = 1
earth_mass = 6.42e23
uni_grav_constant = 6.67e-11
x = 1201.730368
y = 0
z = 0
x_vector = [x, y, z]
position = np.array(x_vector)
v_vector = [0, 250000, 0]
velocity = np.array(v_vector)

# simulation time, timestep and time
t_max = 100
dt = 0.0005
t_array = np.arange(0, t_max, dt)

# initialise empty lists to record trajectories
x_list = []
y_list = []
z_list = []
v_list_x = []
v_list_y = []
v_list_z = []

# Euler integration
for t in t_array:

    # append current state to trajectories
    x_list.append(position[0])
    y_list.append(position[1])
    z_list.append(position[2])
    v_list_x.append(velocity[0])
    v_list_y.append(velocity[1])
    v_list_z.append(velocity[2])

    # calculate new position and velocity
    a = - (uni_grav_constant * earth_mass / position.dot(position)) * (position / np.sqrt(position.dot(position)))
    position = position + dt * velocity
    velocity = velocity + dt * a



# Verlet method 



# initialise conditions again
m = 1
earth_mass = 6.42e23
uni_grav_constant = 6.67e-11
x = 1201.730368
y = 0
z = 0
x_vector = [x, y, z]
position = np.array(x_vector)
v_vector = [0, 250000, 0]
velocity = np.array(v_vector)


# simulation time, timestep and time
t_max = 100
dt = 0.0005
t_array = np.arange(0, t_max, dt)

# initialise empty lists for Verlet integrator this time

x_verlet_list = []
y_verlet_list = []
z_verlet_list = []
v_verlet_list_x = []
v_verlet_list_y = []
v_verlet_list_z = []

# initialise the first x(t-dt)
# as initial velocity is positive, the initial x(t-dt) must be non-zero and negative 
# we extrapolate backwards using the initial velocity condition
position_previous = position - dt * velocity

# Verlet integrator with same initial conditions and sample size 
for t in t_array:

    # append current state to trajectories 
    x_verlet_list.append(position[0])
    y_verlet_list.append(position[1])
    z_verlet_list.append(position[2])
    v_verlet_list_x.append(velocity[0])
    v_verlet_list_y.append(velocity[1])
    v_verlet_list_z.append(velocity[2])


    # calculate new position x(t+dt) and velocity v(t+dt)

    a = - (uni_grav_constant * earth_mass / position.dot(position)) * (position / np.sqrt(position.dot(position)))
    position_current = position
    position = (2 * position_current) - position_previous + (dt * dt) * a
    velocity = (1/dt) * (position - position_current)

    position_previous = position_current


# position
plt.figure(f"Position from Euler's method")
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, x_list, label='x (m)')
plt.plot(t_array, y_list, label='y (m)')
plt.plot(t_array, z_list, label='z (m)')
plt.legend()
plt.show()

plt.figure(f"Position from Verlet's method")
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, x_verlet_list, label='x (m)')
#plt.plot(t_array, y_verlet_list, label='y (m)')
plt.plot(t_array, z_verlet_list, label='z (m)')
plt.legend()
plt.show()


# velocity
plt.figure(f"Velocity from Euler's method")
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, v_list_x, label='x (m/s)')
plt.plot(t_array, v_list_y, label='y (m/s)')
plt.plot(t_array, v_list_z, label='z (m/s)')
plt.legend()
plt.show()

plt.figure(f"Velocity from Verlet's method")
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, v_verlet_list_x, label='x (m/s)')
#plt.plot(t_array, v_verlet_list_y, label='y (m/s)')
plt.plot(t_array, v_verlet_list_z, label='z (m/s)')
plt.legend()
plt.show()

