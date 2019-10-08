# Sensors {#part:sensor_introduction status=ready}

Sensors are necessary for a robot to perceive its environment. Each sensor allows the robot to know something more about the world around it based on the type of data that the sensor provides. This means that a robot's understanding of its surroundings is limited by the types and numbers of sensors which provide information to the robot. When designing a robot, a roboticist must select sensors that will allow the robot to perceive enough information to perform its intended task. Since multiple sensors can be used to provide the same data (at varying accuracies), the roboticist must take into account the level of precision required for the robot's intended task, as well as the cost limitations of the sensors, and computational requirements. For example, when choosing the sensors for your drone, the goal was to achieve the lowest cost flying autonomous learning platform.

## Learning Objectives

After taking this module, students should be able to describe the
sensors used on the drone, how they work, and their function.
Specifically, we will cover the IR sensor, which is used to estimate
height, how it works, and how to calibrate it.  Then we will cover the
Inertial Measurement Unit (IMU), which is used to measure angular
velocity and linear accelleration.  Finally we will interface with the
camera, which is used to measure planar velocity and global position.
This module focuses on the interfacing necessary to obtain raw sensor
readings, calibrate them into metric units if necessary, and publish
the readings on the appropriate ROS topics.
