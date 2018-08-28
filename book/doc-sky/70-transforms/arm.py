import numpy as na
from time import time

import matplotlib.pyplot as plt
import math
import matplotlib.animation as animation

def deg2rad(deg):
    return deg / 180.0 * math.pi

class Arm2D:
    def __init__(self, axes):

        # the angle at each joint, in the coordinate frame of the joint.
        self.joints = na.array([deg2rad(180), deg2rad(0.0), deg2rad(0)])

        # the length of each joint.
        self.lengths = [1.0, 3.0, 2.0]

        # the origin of the coordinate system in x, y, theta.
        self.origin = [0.0, 0.0, 0.0]

        self.update()
        self.press = None
        self.axes = axes


    def drawFrame(self, x, y, theta):
        """
        Draw the coordinate frame given x, y, theta in the base frame.
        Returns the artists, needed for matplotlib animations.
        """
        artists = []

        artists.extend(self.axes.plot([x, x + math.cos(theta)],
                                      [y, y + math.sin(theta)], 'o-', lw=2, c='r'))
        artists.extend(self.axes.plot([x, x + math.cos(theta + math.pi/2)],
                                      [y, y + math.sin(theta + math.pi/2)], 'o-', lw=2, c='g'))
        
        return artists


    def update(self):
        """
        Update the robot's state given the joint angles.  Call after
        changing the joint angles.
        """

        jointGlobals = []  # contains the global pose (x,y,theta) at each joint.

        here = na.array(self.origin)
        x, y, theta = here
        jointGlobals.append(na.copy(here))
        jointTransforms = [na.copy(here)]  # contains the transform from joint i to i + 1

        # TODO: fill in joint globals and joint transforms so you can
        # drive the robot joint angles from the keyboard as we did in class. 

        self.jointGlobals = na.array(jointGlobals)
        self.jointTransforms = jointTransforms

 
    def step(self, t):
        """Update the world state.  Placeholder if we want to add gravity or
dynamics later.

        """
        pass

    def animate(self, fig):
        """
        Animation callback.  Called at 25hz after state changes.  
        """
        
        t = time()
        self.step(t)
        self.update()
        artists = []
        artists.extend(self.axes.plot(self.jointGlobals[:,0], self.jointGlobals[:,1], 'o-', lw=2, c='k'))

        
        here = na.array([0.0, 0.0, 0.0])
        for i in range(len(self.jointGlobals)):
            here = self.jointGlobals[i]
            x, y, theta = here
            #artists.extend(self.drawFrame(x, y, theta))


        here = na.array([0.0, 0.0, 0.0])        
        for i in range(len(self.jointTransforms)):
            here += self.jointTransforms[i]
            x, y, theta = here
            #artists.extend(self.drawFrame(x, y, theta))
            
        return artists
        
    def on_press(self, event):
        """
        On mouse press callback.  Placeholder for IK from mouse.
        """
        if event.inaxes != self.axes: 
            return
        self.press = event.xdata, event.ydata

    def on_release(self, event):
        """
        On mouse release callback.  Placeholder for IK from mouse.
        """
        self.press = None

    def on_motion(self, event):
        """
        On mouse motion callback.  Placeholder for IK from mouse.
        """
        if self.press is None: 
            return
        if event.xdata == None or event.ydata == None:
            return

        xpress, ypress = self.press
        dx = event.xdata - xpress
        dy = event.ydata - ypress
        
        print("dx, dy", dx, dy)

    def on_key_press(self, event):
        """
        On key press.   Handles keyboard-based joint control.
        """
        print("event", event.key)
        print(event.key == 'j')
        if event.key == 'j':
            self.joints[0] += 0.1
        elif event.key == 'l':
            self.joints[0] -= 0.1

        if event.key == 'a':
            self.joints[1] += 0.1
        elif event.key == 'd':
            self.joints[1] -= 0.1


        if event.key == 'i':
            self.joints[2] -= 0.1

        if event.key == 'k':
            self.joints[2] += 0.1



    def on_key_release(self, event):
        pass

def main():
    fig = plt.figure()
    ax = fig.add_subplot(111, autoscale_on=False, xlim=(-15, 15), ylim=(-15,15))
    arm = Arm2D(ax)
    ax.set_aspect('equal')
    #ax.axis((-10, 10, -10, 10))
    plt.title("2D Arm")
    fig.canvas.mpl_connect('button_press_event', arm.on_press)
    fig.canvas.mpl_connect('button_release_event', arm.on_release)
    fig.canvas.mpl_connect('motion_notify_event', arm.on_motion)
    fig.canvas.mpl_connect('motion_notify_event', arm.on_motion)

    fig.canvas.mpl_connect('key_press_event', arm.on_key_press)
    fig.canvas.mpl_connect('key_release_event', arm.on_key_release)
    ax.grid()
    
    ani = animation.FuncAnimation(fig, arm.animate,
                                  interval=10, blit=True)
    print(ani)
    plt.show()
    

if __name__ == "__main__":
    main()
