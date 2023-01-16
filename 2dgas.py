import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib import patches


#main process
'''
Boundaries at x => 0 - 100
              y => 0 - 100
'''
radius = 5 # meters
delta_t = 20e-3 #seconds

position = 100*np.random.rand(10,2) # m
velocity = 200*np.random.rand(10,2) # m/s

position_min = np.array([0,0]) #[x_min,y_min]
position_max = np.array([100,100]) #[x_max,y_max]

particles = [0,1,2,3,4,5,6,7,8,9]


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

def do_elastic_collision(p1,p2):
    return 

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
        # print(position)
        # print(p1,p2)
        velocity[p1,0], velocity[p2,0] = velocity[p2,0], velocity[p1,0]
        velocity[p1,1], velocity[p2,1] = velocity[p2,1], velocity[p1,1]

    return  None

#animation
fig, ax = plt.subplots()
ax.set_xlim(position_min[0], position_max[0])
ax.set_ylim(position_min[1], position_max[1])
circles = [patches.Circle((0,0), radius=radius, color='green') for p in particles]
[ax.add_patch(circle) for circle in circles]
labels = [0,1,2,3,4,5,6,7,8,9]

for p in particles:
    labels[p] = ax.text(position[p,0], position[p,1], p, size=12)

def animation_frame(i):
    global position, velocity, labels
    do_timestep(i)

    # print(position)

    for p,circle in zip(particles,circles):
        circle.center = position[p,0], position[p,1]
        labels[p].remove()
        labels[p] = ax.text(position[p,0], position[p,1], p, size=12)

animation = FuncAnimation(fig, func=animation_frame, frames=np.arange(0,10,0.1), interval=delta_t*1e3)

plt.show()