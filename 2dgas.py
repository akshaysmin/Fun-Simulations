import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib import patches
import seaborn as sns



#main process
'''
Boundaries at x => 0 - 100
              y => 0 - 100
'''
SHOW_DIST = True
SHOW_ANIM = True

radius = 1 # meters
delta_t = 20e-3 #seconds
NUM_PARTICLES = 200
HIST_BINS = 50

position = 100*np.random.rand(NUM_PARTICLES,2) # m
# velocity = 200*(np.random.rand(NUM_PARTICLES,2)-0.5) # m/s
velocity = 20*np.ones((NUM_PARTICLES,2)) # m/s

# v*delta_t = 0.4

position_min = np.array([0,0]) #[x_min,y_min]
position_max = np.array([100,100]) #[x_max,y_max]

particles = [i for i in range(NUM_PARTICLES)]

def do_hard_wall_reflection(position_min, position_max, position, velocity):
    velocity = np.where((position>=position_max) | (position<=position_min), -velocity, velocity)

    position = np.where(position>=position_max, 2*position_max - position, position)
    
    position = np.where(position<=position_min, 2*position_min - position, position)

    return position, velocity

def recursive_collision_finder(particle1, other_particles, num_other_particles):
    # particle1,other_particles are sorted in the order of their x-coordinate
    colliding_pairs = []

    for particle2 in other_particles:
        if np.abs(position[particle1,0]-position[particle2,0])<=2*radius: # check if x-coordinates are nearby
            if np.abs(position[particle1,1]-position[particle2,1])<=2*radius: # check if y-coordinates are nearby
                # check if particles are approaching each other
                r21 = position[particle2] - position[particle1]
                v21 = velocity[particle2] - velocity[particle1]
                v21_axial = np.dot(v21, r21)
                if v21_axial<0:
                    colliding_pairs.append((particle1, particle2))
        else:
            break
    
    if num_other_particles>1:
        colliding_pairs += recursive_collision_finder(other_particles[0], other_particles[1:], num_other_particles-1)

    return colliding_pairs

def do_timestep(i):
    global position, velocity

    #velocity
    position = position + velocity*delta_t

    #reflection at hard bondaries
    position, velocity = do_hard_wall_reflection(position_min, position_max, position, velocity)

    # #elastic collision with other particles - O(nlog(n))
    x_ordered_particle_index = np.argsort(position[:,0])
    colliding_pairs = recursive_collision_finder(x_ordered_particle_index[0], x_ordered_particle_index[1:], len(x_ordered_particle_index)-1)

    # #modify this
    for p1,p2 in colliding_pairs:
        # collision physics goes here
        v20 = velocity[p2] - velocity[p1]
        r20 = position[p2] - position[p1]
        v2a = (np.dot(v20,r20)/np.dot(r20,r20))*r20

        velocity[p2] += -v2a
        velocity[p1] +=  v2a

    return  None

#animation
fig = plt.figure(figsize=(8,4))
ax = fig.add_subplot(1,2,1)
ax2 = fig.add_subplot(1,2,2)
ax.set_xlim(position_min[0], position_max[0])
ax.set_ylim(position_min[1], position_max[1])
circles = [patches.Circle((0,0), radius=radius, color='green') for p in particles]
[ax.add_patch(circle) for circle in circles]
# labels = [i for i in particles]

# for p in particles:
#     labels[p] = ax.text(position[p,0], position[p,1], p, size=12)

speed = np.linalg.norm(velocity, axis=1)
sns.kdeplot(speed, ax=ax2)
ax2.set_xlabel('speed (m/s)')
ax2.set_ylabel('probability')

def animation_frame(i):
    global position, velocity, labels
    do_timestep(i)

    if SHOW_DIST:
        speed = np.linalg.norm(velocity, axis=1)
        ax2.clear()
        sns.kdeplot(speed, ax=ax2)
        ax2.set_xlabel('speed (m/s)')
        ax2.set_ylabel('probability')

    if SHOW_ANIM:
        for p,circle in zip(particles,circles):
            circle.center = position[p,0], position[p,1]
            # labels[p].remove()
            # labels[p] = ax.text(position[p,0], position[p,1], p, size=12)

animation = FuncAnimation(fig, func=animation_frame, frames=np.arange(0,10,0.1), interval=delta_t*1e3)

plt.show()