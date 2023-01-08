import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

#main process
'''
Boundaries at x => 0 - 100
              y => 0 - 100
'''
x = 100*np.random.rand(10)
y = 100*np.random.rand(10)

vx = 2*np.random.rand(10)
vy = 2*np.random.rand(10)

x_min, x_max = 0,100
y_min, y_max = 0,100

radius = 20

def do_timestep(i, x,y, vx,vy):

    #velocity
    x += vx
    y += vy

    #reflection at hard bondaries
    vx = np.where((x>=x_max) | (x<=x_min), -vx, vx)
    vy = np.where((y>=y_max) | (y<=y_min), -vy, vy)

    x = np.where(x>=x_max, 2*x_max - x, x)
    y = np.where(y>=y_max, 2*y_max - y, y)
    
    x = np.where(x<=x_min, 2*x_min - x, x)
    y = np.where(y<=y_min, 2*y_min - y, y)

    #elastic collision with other particles

    return  x,y, vx,vy

#animation
fig, ax = plt.subplots()
ax.set_xlim(0,100)
ax.set_ylim(0,100)
line, = ax.plot(0,0, 'bo',  markersize=radius)

def animation_frame(i):
    global x,y, vx,vy
    x,y, vx,vy = do_timestep(i, x,y, vx,vy)
    print(x,y)

    line.set_xdata(x)
    line.set_ydata(y)

    return line,

animation = FuncAnimation(fig, func=animation_frame, frames=np.arange(0,10,0.1), interval=10)

plt.show()