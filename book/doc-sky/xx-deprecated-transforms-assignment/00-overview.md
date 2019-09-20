# Transforms {#part:transforms status=ready}

This unit focuses on transforms in robotics. In our experience, a
significant fraction of effort in robotics programming is spent making
sure transforms are correct. If you have an incorrect transform
between a sensor and the robot's body, it is much harder to use the
sensor information. Inaccurate transforms directly translates to less
precise information from the sensor. A useful reference on this topic
is [](#bib:diebel2006representing).

The process of finding a transform between the robot's body and a sensor is
called [extrinsic
calibration](https://en.wikipedia.org/wiki/Camera_resectioning#Extrinsic_parameters).



In this assignment we will practice using transformation matrices to compute
the location of points in different coordinate systems as well as methods for
performing extrinsic calibration. This topic is closely related to graphics,
which uses the full power of a transformation matrix to shear, reproject etc.
In robotics, the main activity is translation and rotation of rigid bodies. For
example, after driving forward at 2 meters per second for 1 second, we want to
estimate the robot's new position. Or for example, after yawing at 10 degrees
per second for 5 seconds, we want to find the robot's updated rotation in the
global space.
