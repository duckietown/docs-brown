# Assignment {#sensors-assignment status=ready}

## IMUs (20 points)

A robot is equipped with an IMU that returns linear acceleration at
$10\mbox{hz}$ as: $$\begin{bmatrix}x\\y\\z\end{bmatrix}$$.

Assume the robot is at rest, and that the IMU calibrated so that $z$
is downwards and $x$ and $y$ are in the horizontal plane.

1. Assume the robot is at the origin with a velocity of $0$ at the start of
   question 1. The robot receives the following sensor readings:
   - $$\begin{bmatrix}1\\0\\9.8\end{bmatrix}$$ for $3$ seconds
   - $$\begin{bmatrix}-2\\0\\9.8\end{bmatrix}$$ for $1.5$ seconds.
   What was the robot's motion? Specifically:

   1.1. What happened for the first $3$ seconds?
   1.2. What was the robot's position and velocity after the first $3$ seconds?
   1.3. What happened during seconds $3$ through $4.5$?
   1.4. What was the robot's position and velocity after $4.5$ seconds?

2. The robot receives the following sensor readings: 
   - $$\begin{bmatrix}0\\0\\9.8\end{bmatrix}$$ for $10$ seconds.

   Is the robot moving or still? How do you know?

3. Why does the $z$ term read $9.8$?

## Range Finders (20 points)

A robot is equipped with an IR range finder. This sensor returns a measurement
of distance to an IR opaque obstacle. Read the page about IR sensors
[here](http://education.rec.ri.cmu.edu/content/electronics/boe/ir_sensor/1.html).

3.1. Will this sensor work outside in the sun? Why or why not?

3.2. The sensor returns an unsigned byte between $0$ and $255$ that can be
     interpreted as distance in meters $d$, so that $d = r \times K$ where $d$
     is the distance and $r$ is the raw sensor value and $K$ is a calibration
     factor. How could you calibrate the sensor so that you can measure
     distance from the robot in meters?

3.3. To calibrate the sensor you place obstacles in front of the sensor and
     record the following values: $2\mbox{m}$ returns $27$. What is the value
     for $K$?

3.4. Your sensor reads $75$. How far away is the obstacle in meters? Use the
     value $K = 0.5$.

3.5. You calibrate your sensor again by taking multiple readings at different
     distances. $1\mbox{m}$ returns $100$, $3\mbox{m}$ returns $297$, and
     $6\mbox{m}$ returns $605$ Now how would you estimate $K$? Keep in mind
     that the equation $d - r \times K$ is linear, and that there may be some
     variance in the sensor readings.

## Research a sensor (20 points)

Pick a sensor from the following choices: 
- Raspberry PI Camera
- Hokuyo LIDAR 
- Velodyne Puck 

Research what the sensor does and answer the following questions: 
4.1. Is it passive or active? 
4.2. What is its range? 
4.3. Write four to five sentences describing the sensor and how it works.

## Detecting objects on a conveyor belt (20 points)

You are designing a robot to detect an object's location on an assembly line so
that it can pick it up. It will always be picking the same object off of
a moving conveyor belt. Assume that the IR sensor is mounted to the gripper of
the robot arm. If you would like a visual example, imagine attaching an IR
sensor to Baxter's hand, pointing the same direction as Baxter's gripper.

5.1. Describe how you would use an IR sensor to carry out this task. Write what
     assumptions you need to make about the object's location and the
     environment for this solution to work.

5.2. Describe how you would use an RGBD camera to carry out this task. 

5.3. Describe in both cases how you would grasp the object given its location.

## Avoiding collisions (20 points)

You would like to modify the drone so that it avoids collisions with objects
when it flies, such as people, walls, and trees.

6. Describe how you would use an IR sensor to carry out this task. Make sure to answer the following questions:
6.1. How many sensors would you use?
6.2. How would you mount them?
6.3. How would use use the data?
6.4. What assumptions about the environment must you make in order for this
     solution to work?

## Handin

When you are done, use [this link]() to create your Sensors Github Repo. Commit
and push the relevant files:
- answers.txt
