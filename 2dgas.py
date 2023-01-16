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
num_particles = 2

position = 100*np.random.rand(10,2) # m
velocity = 50*np.random.rand(10,2) # m/s

position_min = np.array([0,0]) #[x_min,y_min]
position_max = np.array([100,100]) #[x_max,y_max]

particles = np.arange(num_particles)


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

def do_elastic_collision(r1, r2, v1, v2):

    #F.O.R. origin and velocity
    r0, v0 = r1, v1

    #before collision
    v20 = v2 - v0
    r20 = r2 - r0

    v20_axial = np.dot(v20,r20)*r20/np.dot(r20,r20)
    v20_nonaxial = v20 - v20_axial

    v10 = 0

    #after collision
    v20 = v20_nonaxial
    v10 = v20_axial

    v1 = v0 + v10
    v2 = v0 + v20

    #calculating final position
    dt1 = (np.linalg.norm(r20)-2*radius)/np.linalg.norm(v20_axial) #before collision
    dt2 = delta_t - dt1  #after collision

    r20 = v20_axial*dt1 + v20_nonaxial*dt2
    r10 = 0*dt1 + v20_axial*dt2

    r1 = r0 + r10
    r2 = r0 + r20

    return r1, r2, v1, v2

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
        # velocity[p1,0], velocity[p2,0] = velocity[p2,0], velocity[p1,0]
        # velocity[p1,1], velocity[p2,1] = velocity[p2,1], velocity[p1,1]
        position[p1], position[p2], velocity[p1], velocity[p2] = do_elastic_collision(position[p1]-velocity[p1]*delta_t, position[p2]-velocity[p2]*delta_t, velocity[p1], velocity[p2])

    return  None

#animation
fig, ax = plt.subplots()
ax.set_xlim(position_min[0], position_max[0])
ax.set_ylim(position_min[1], position_max[1])
circles = [patches.Circle((0,0), radius=radius, color='blue') for p in particles]
[ax.add_patch(circle) for circle in circles]
labels = [0]*num_particles
for p in particles:
    labels[p] = ax.text(position[p,0], position[p,1], p, size=12)

def animation_frame(i):
    global labels
    do_timestep(i)

    # print(position)

    for p,circle in zip(particles,circles):
        circle.center = position[p,0], position[p,1]
        labels[p].remove()
        labels[p] = ax.text(position[p,0], position[p,1], p, size=12)

animation = FuncAnimation(fig, func=animation_frame, frames=np.arange(0,10,0.1), interval=delta_t*1e3)

plt.show()