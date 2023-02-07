import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

#main process
#initiation
'''
Boundaries at x => 0 - 100
              y => 0 - 100
              z => 0 - 100
'''
radius = 2 # meters
delta_t = 10e-3 #seconds
NUM_PARTICLES = 10

position = 100*np.random.rand(NUM_PARTICLES,3) # m
velocity = 200*np.random.rand(NUM_PARTICLES,3) # m/s

position_min = np.array([0,0,0]) #[x_min,y_min,z_min]
position_max = np.array([100,100,100]) #[x_max,y_max,z_max]

particles = [i for i in range(NUM_PARTICLES)]

#functions
def do_hard_wall_reflection(position_min, position_max, position, velocity):
    velocity = np.where((position>=position_max) | (position<=position_min), -velocity, velocity)

    position = np.where(position>=position_max, 2*position_max - position, position)
    
    position = np.where(position<=position_min, 2*position_min - position, position)

    return position, velocity

#simulation
def do_timestep(i):
    global position, velocity
    
    #velocity
    position = position + velocity*delta_t

    #reflection at hard bondaries
    position, velocity = do_hard_wall_reflection(position_min, position_max, position, velocity)
    
    return None

#animation
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(position_min[0], position_max[0])
ax.set_ylim(position_min[1], position_max[1])
ax.set_zlim(position_min[2], position_max[2])
scat = ax.scatter(position[:,0], position[:,1], position[:,2])

def animation_frame(i):
    global position, velocity
    do_timestep(i)

    scat._offsets3d = position[:,0], position[:,1], position[:,2]

animation = FuncAnimation(fig, func=animation_frame, frames=np.arange(0,10,0.1), interval=delta_t*1e3)

plt.show()