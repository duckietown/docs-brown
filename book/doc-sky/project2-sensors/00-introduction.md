# Project 2: Sensor Interfacing {#sensors-project status=draft}

## Introduction
Sensors are necessary for a robot to perceive its environment. A Robots knowledge of its surroundings are, therefore, limited by the types and numbers of sensors which provide information to the robot. The information sensors provide is meaningless unless the data can be interpreted by the robot and converted into useful metrics.

## Your Robot's Sensors
The PiDrone is equipped with three sensors: an inertial measurement unit (IMU), an infrared sensor, and a downward facing camera. From these sensors, the drone is equiped with enough knowledge of it's environment to control it's flight and fly autonomously.

### IMU
An IMU is an electronic device that uses accelerometers and gyroscopes to measure forces (via accelerations) and angular rates acting on a body. The IMU on the PiDrone is a component of the flight controller. Data provided by the IMU used is used by the flight controller itself to stabilize the flight.

### Camera
Since perceived velocities are based on the distance from which they are perceived, the height of the drone must be known to scale the motion vectors. To get this data, you will subscribe to the state of the drone which will be published by
### Infrared
