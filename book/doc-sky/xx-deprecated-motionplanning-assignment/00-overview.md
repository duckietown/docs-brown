# Motion Planning {#part:motionplanning status=ready}

This unit focuses on the problem of motion planning for robotics. This
is the problem of moving through space without colliding. It typically
abstracts sensing and perception and assumes perfect ability to move
in the space. The problem is to find a trajectory through the space
for the robot (which may be a drone, a vehicle or an arm) that avoids
collisions.

Formally, the input to motion planning is the model of the robot, a
model of obstacles in the environment, and a start state, and a goal
state.  The output is a trajectory through space that causes the robot
to move from start state to goal state without colliding with
obstacles in its environment.  A real-life example of motion planning
is when a person parallel parks a car.  This trajectory is not
obvious, and takes time to learn, because of the car's movement
constraints.

Avoiding obstacles is a key part of robotics.  The Skydio drone's
advance over the state-of-the-art was its ability to accurately detect
and avoid obstacles in its environment.  In our work so far with the
drone, we have modeled the robot as a point, and ignored obstacles.
Indeed, the drone does not have sensors pointed in any direction but
downwards and has no awareness of obstacles in its environment.
However we will model this problem by creating virtual obstacles that
the drone will avoid as it flies.

A second important domain for motion planning is robotic arms.  A
robotic arm is modeled as a number of joints and arm geometry.  Each
joint is parameterized as a joint angle (and in general, joint
velocity).  This parameter can be set to move the arm to a particular
joint, and read using joint encoders.  Given the joint states and arm
geometry, we can compute the end effector pose.  This problem is
called forward kinematics and has a closed form solution.  We would
also like to solve the inverse problem: given an end effector pose we
would like to find joint states that result in the end effector
attaining that position.  This problem is called inverse kinematics
and does not have a closed form solution.



