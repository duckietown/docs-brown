# Assignment {#motionplanning-assignment status=ready}

## Configuration Space

1.1. Imagine an arm moving in three dimensions with three joints, $\theta_1$ and
     $\theta_2$, and $\theta_3$. What is the configuration space?

1.2. Imagine a $1.5\mbox{m}$ diameter circular robot moving in a room with four
     walls, one at $x=-20$, a second at $x=30$, a third at $y=-10$ and a fourth
     at $y=0$. The robot's position is indexed at its center and it is
     omnidirectional (that is, it can move in any direction without turning).
     All these coordinates are in meters. What is the robot's configuration
     space?

## Manual Motion Planning

The following problems will use the arm.py code(in the Github Classroom). First we ask you to
perform motion planning manually. This is surprisingly useful for understanding
the robot's degrees of freedom and what it can do. It is also notable that
manual motion planning is how the Canada Arm on the International Space Station
is used today. They check each plan manually and tweak it by hand, because
collisions between the arm and the station would be catastrophic.

2. Move the arm from its start location to as close as possible to $x=8, y=0$
   so that it does not collide with the circle. Describe how you had to move
   the arm in words (for example using words like clockwise and counter
   clockwise) and also submit four images showing the sequence of positions the
   arm went through.

## Implementing the RRT

Implement the skeleton methods for a 2d RRT in python following [this
paper](https://cs.brown.edu/courses/cs1951r/assignments/motionplanning/rrtpaper.pdf) and rrt.py(found in the Github Classroom).

<!-- ![](rrt.png) -->

Use the RRT to find a motion plan for a 2d robot for the same point as above.
The `r` key starts running the RRT to expand it, and then pressing `r` again
stops it from expanding.

3.1. First run the RRT for a second or two. Describe in words what the RRT
     does, and submit four images showing its intermediate progress.

3.2. Now run it for a long time, 10 or 20 seconds. Describe in words
     what the RRT does, and submit four images showing its intermediate
     progress.

3.3. The motion of the robot with the RRT should differ from your planned
     motion/ideal motion. Does it find the shortest path of the arm to the
     goal? Why or why not?

3.4. How does the motion of the robot with the RRT when run for a short time
     differ from when run for a long time?

## Handin

When you are done, use [this link](https://classroom.github.com/a/hhOhHYYd) to create your Motion Planning Github
Repo. Commit and push the relevant files:

- answers.md
- All images, named by question number and
- Any python files that you implemented your RRT
