# Assignment {#transforms-assignment status=ready}

## Translations (33 points)

A translation can be represented as a tuple $<x, y, z>$ corresponding to the
amount a point moves in each direction. *Write the answers to the following
questions in `answers.md`.*

1. The robot is at the origin, $\begin{bmatrix}0\\0\\0\end{bmatrix}$. It drives
   forward for 5 seconds at 2.5 meters per second. So its velocity is
   $\begin{bmatrix}0\\2.5\\0\end{bmatrix}$. How many meters has the robot
   driven, and what is its new pose? Represent its new pose as a 3-vector.

2. Now the robot turns. (Ignore rotational pose for this problem.) It drives
   forward at 1 m/s for 10 s, so its velocity is $\begin{bmatrix}\sqrt{2}, 1,
   0\end{bmatrix}$. How many meters has the robot driven, and what is its new
   pose? Represent its new pose as a 3-vector.

3. To represent translation in a transformation matrix, we add $1$ to the
   vector representing the robots pose: $<x, y, z, 1>$. To see why this is
   necessary, consider multiplying a $3\times3$ matrix $T$ by a the vector $<0,
   0, 0>$. Formally:
$$
   \begin{align}
    T \times \begin{bmatrix}0\\0\\0\\
   \end{bmatrix}\end{align}$$

   What is the result? Is it possible to represent a translation in a $3\times3$
   matrix? If so, provide an example. If not, explain why not.

4. To fix this problem, we add an extra entry to the position vector which is
   always $1$. Position at the origin is represented with the vector:

   $$\begin{bmatrix}0\\0\\0\\1\end{bmatrix}.$$

   We can represent a transformation as a matrix $T$, where
   $$
   \begin{bmatrix}
   1 & 0 & 0 & t_x\\
   0 & 1 & 0 & t_y\\
   0 & 0 & 1 & t_z\\
   0 & 0 & 0 & 1
   \end{bmatrix}
   $$

   To translate the robot, one performs the following multiplication: $p'
   = T \times p$. Where $p'$ is the position at the next timestep and $p$ is
   the position at the current timestep.

   For this problem, the robot starts at the origin. It moves right $3$ meters
   in the $x$ direction, flies up in $z$ $0.5$ meters, and then moves backward
   in the $y$ direction $-1$ meter.

   4.1. Draw the robot's trajectory. *Submit this picture as `trajectory.png`.*

   4.2. Write out the transformations as three separate transformation
   matrices.

   4.3. Multiply the matrices together to get one single transform.

   4.4. Multiply the matrix by the robot's position vector to get its new
   location.

   Show all of your work

## Understanding a Point Cloud (33 points)

The robot observes the following point cloud, denoted in the form $$<angle,
distance>$$. For frame of reference, use $$0^{\circ}$$ is off to the robot's
right, $$90^{\circ}$$ is straight ahead, and $$180^{\circ}$$ is to the robot's
left:
$$
[(0, 1), (45, 5), (90, 2), (180, 4), (270, 3)].
$$

For the following problems, assume this is all points the robot can see in the
world, so do not worry about new things that might be out of frame.

5.1. Draw a graph with the robot at $(0, 0)$ facing forward along the $y$ axis
     and draw the points that the robot sees. *Submit this graph as an image
     titled `question_5_1.png`.*

5.2. Now assume the robot drives forward one meter. Provide the new sensor
     readings, assuming perfectly accurate sensors and a perfectly accurate
     motion model. Draw them in a new version of the graph above. *Submit this
     graph as an image titled `question_5_2.png`.*

5.3. Now assume the robot rotates in place by 30 degrees. Draw what it would
     see. *Submit this graph as an image titled `question_5_3.png`.*

## Robotic Arms (33 points)

A 2D linear robot has three joints as depicted below. The distance between the
first and second joint is 1m, the second and third is 3m, and the third and end
is 2m.

6.1. Implement the transformations in the skeleton code (provided in the GitHub Classroom), so that the
   arm joints move correctly and the base transformations are drawn. After you
   have implemented the transforms correctly, you should be able to see the
   arm's state as shown in the example image below. You should be able to move
   each joint with 'i', 'k', 'j', 'l', 'a', and 'd' and see the arm update its
   state. We recommend drawing the trigonometry out on paper before
   implementing! When the joint angles are $<0, 0, 0>$, the arm should be
   pointed horizontally along the X axis.

6.2. Where is the arm when the joint angles are $<0, 0, 0>$? Give the position of
   each joint in the base coordinate system.

6.3. Provide joint angles that result in the end effector of the arm being at
   location $<-1, -1>$.

6.4. Is the positioning in question 6.3. possible in real life? Why or why not?

## Handin

When you are done, use [this link]() to create your Transforms GitHub Repo.
Commit and push the relevant files:

- answers.md
- question_5_1.png
- question_5_2.png
- question_5_3.png
