# Sensors {#part:sensors status=ready}

## Introduction
Sensors are necessary for a robot to perceive its environment. A Robots knowledge of its surroundings are, therefore, limited by the types and numbers of sensors which provide information to the robot. The information sensors provide is meaningless unless the data can be interpreted by the robot and converted into useful metrics.

## Your Robot's Sensors
The PiDrone is equipped with three sensors: an inertial measurement unit (IMU), an infrared sensor, and a downward facing camera. From these sensors, the drone is equiped with enough understanding of it's environment to control it's flight and fly autonomously.

### IMU
An IMU is a device that uses accelerometers and gyroscopes to measure forces (via accelerations) and angular rates acting on a body. The IMU on the PiDrone is a built-in component of the flight controller. Data provided by the IMU used is used by the flight controller itself to stabilize the flight, as well as by the state estimator which you will be implementing in the next project to better.

### Infrared
A range sensor is any sensor that measures the distance to an object. There are two main types that are used on quadcopters, ultrasonic, and infrared. For both sensors, a wave is emited from one element of the sensor, and received by the other. The time taken for the wave to be emited, reflected, and be absorbed by the second sensor allows the range to be calculated. Your drone utilizes an infrared sensor becuase it is more accurate and has a better range than the ultrasonic. The IR sensor will be used to measure the height of the drone.

### Camera
Each drone is equipped with a single Arducam 5 megapixel 1080p camera. The camera is used to measure motion in the planar directions. We face this camera down at the ground to measure  x, y, and yaw velocities of the drone using optical flow vectors that are extracted from the camera images. This is a lightweight task, meaning that it does not require much computation, because these vectors are already calculated for image compression. We also use the camera to estimate the relative position of the drone by estimating the rigid transformations between two images.
