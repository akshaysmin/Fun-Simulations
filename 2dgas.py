import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

#main process
'''
Boundaries at x => 0 - 100
              y => 0 - 100
'''
radius = 5

x = 100*np.random.rand(10)
y = 100*np.random.rand(10)

vx = 2*np.random.rand(10)
vy = 2*np.random.rand(10)

x_min, x_max = 0,100
y_min, y_max = 0,100

def do_hard_wall_reflection(x_min,x_max, y_min,y_max, x,y, vx,vy):
    vx = np.where((x>=x_max) | (x<=x_min), -vx, vx)
    vy = np.where((y>=y_max) | (y<=y_min), -vy, vy)

    x = np.where(x>=x_max, 2*x_max - x, x)
    y = np.where(y>=y_max, 2*y_max - y, y)
    
    x = np.where(x<=x_min, 2*x_min - x, x)
    y = np.where(y<=y_min, 2*y_min - y, y)

    return x,y, vx,vy

def recursive_collision_finder(particle1, other_particles, num_other_particles, mode='x'):
    colliding_pairs = set()

    for particle2 in other_particles:
        if mode =='x':
            if np.abs(x[particle1]-x[particle2])<=2*radius:
                colliding_pairs.add((particle1, particle2))
            else:
                break
        elif mode =='y':
            if np.abs(y[particle1]-y[particle2])<=2*radius:
                colliding_pairs.add((particle1, particle2))
            else:
                break
    
    if num_other_particles>1:
        colliding_pairs.update(recursive_collision_finder(other_particles[0], other_particles[1:], num_other_particles-1, mode='x'))

    return colliding_pairs

def do_timestep(i, x,y, vx,vy):

    #velocity
    x += vx
    y += vy

    #reflection at hard bondaries
    x,y, vx,vy = do_hard_wall_reflection(x_min,x_max, y_min,y_max, x,y, vx,vy)

    #elastic collision with other particles - O(nlog(n))
    x_ordered_particle_index = np.argsort(x)
    x_collisions = recursive_collision_finder(x_ordered_particle_index[0], x_ordered_particle_index[1:], len(x)-1, mode='x')
    y_ordered_particle_index = np.argsort(y)
    y_collisions = recursive_collision_finder(y_ordered_particle_index[0], y_ordered_particle_index[1:], len(y)-1, mode='x')
    colliding_pairs = x_collisions & y_collisions

    #modify this
    for p1,p2 in colliding_pairs:
        manhattan_distance1 = np.abs(x[p1]-vx[p1] -x[p2]+vx[p2]) + np.abs(y[p1]-vy[p1] -y[p2]+vy[p2])
        manhattan_distance2 = np.abs(x[p1]-x[p2]) + np.abs(y[p1]-y[p2])
        if manhattan_distance2 < manhattan_distance1 :
            vx[p1], vx[p2] = vx[p2], vx[p1]
            vy[p1], vy[p2] = vy[p2], vy[p1]

    return  x,y, vx,vy

#animation
fig, ax = plt.subplots()
ax.set_xlim(x_min,x_max)
ax.set_ylim(y_min,y_max)
line, = ax.plot(0,0, 'bo',  markersize=2*radius)

def animation_frame(i):
    global x,y, vx,vy
    x,y, vx,vy = do_timestep(i, x,y, vx,vy)
    print(x,y)

    line.set_xdata(x)
    line.set_ydata(y)

    return line,

animation = FuncAnimation(fig, func=animation_frame, frames=np.arange(0,10,0.1), interval=5)

plt.show()