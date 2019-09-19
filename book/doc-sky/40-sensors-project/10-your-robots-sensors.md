# Your Robot's Sensors {#sensors-your-robot status=ready}

Your drone is equipped with three sensors: an inertial measurement unit (IMU), an infrared sensor, and a downward facing camera. From these sensors, the drone is equipped with enough understanding of its environment to control its flight and fly autonomously. Each sensor is described below. By interfacing with each of these sensors, you will gain exposure to core robotics concepts including frame conversions, interpreting digital signals, and computer vision.


### Infrared
A range sensor is any sensor that measures the distance to an object. There are two main types that are used on quadcopters: ultrasonic and infrared. For both sensors, a wave is emitted from one element of the sensor and received by the other. The time taken for the wave to be emitted, reflected, and be absorbed by the second sensor allows the range to be calculated. Your drone utilizes an infrared sensor because it is more accurate, less noisy, and has a better range than the ultrasonic range sensor. The IR sensor will be used to measure the height of the drone.


### Inertial Measurement Unit (IMU)
An IMU is a device that uses accelerometers and gyroscopes to measure forces (via accelerations) and angular rates acting on a body. The IMU on the PiDrone is a built-in component of the flight controller. Data provided by the IMU are used by the state estimator, which you will be implementing in the next project, to better understand its motion in flight. In addition, the flight controller board uses the IMU data to stabilize the drone from small perturbations.

The IMU can be used to measure global orientation of roll and pitch
(but not yaw).  This measurement is because it measures accelleration
due to gravity, so it can measure the downward pointing gravity
vector.  However it does not have a global yaw measurement.  Many
drones additionally include a magnetomitor to measure global yaw
according to the Earth's magnetic frame, although our drone does not
have this sensor.

Note that IMU does NOT measure position or linear velocity.  The
acceleration measurements can be integrated (added up over time) to
measure linear velocity, and these velocity estimates can be
integrated again to measure position.  However without some absolute
measurement of position or velocity, these estimates will quickly
diverge.  To measures these properties of the drone, we need to use
the camera as described below.


### Camera
Each drone is equipped with a single Arducam 5 megapixel 1080p camera. The camera is used to measure motion in the planar directions. We face this camera down at the ground to measure  x, y, and yaw velocities of the drone using optical flow vectors that are extracted from the camera images. This is a lightweight task, meaning that it does not require much added computation, because these vectors are already calculated by the Pi's image processor for h264 video encoding. We also use the camera to estimate the relative position of the drone by estimating the rigid transformations between two images.

