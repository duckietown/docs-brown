# Using your Time-of-flight Sensor {#sensors-assignment-tof status=ready}

In this part of the project, you will learn how to estimate the drone's height using its time-of-flight sensor. The drone is equipped with a [VL53L0X](https://learn.adafruit.com/adafruit-vl53l0x-micro-lidar-distance-sensor-breakout), which is used for estimating the distance from the drone to the ground-plane. The sensor outputs a digital signal containing the distance from the sensor and read in by the Raspberry Pi via the GPIO pin using the associated [Python library](https://docs.circuitpython.org/projects/vl53l0x/en/latest/). The voltage value corresponds to distance, but we are going to need to do some work to convert it to real-world units.

**Setup**

Change to `~/catkin_ws/src` on your drone, and then `git clone https://github.com/h2r/project-sensors-implementation-yourGithubName`.  You should create a github personal access token for your drone to make this possible.  It only needs permissions to read and write to repositories. 


Change directories into `~/catkin_ws/src/project-sensors-implementation-yourGithubName`.  You can `rosrun project-sensors-yourGithubName student_tof_pub.py`.  You may stop _student_tof_pub.py_ with ctrl-c, edit it within that tab, and then re-run `rosrun project-sensors-yourGithubName student_tof_pub.py` to test your changes.


## Problem 1: Publish your TOF Reading
In _student_tof_pub.py_, fill in the minimum range, maximum range, and current range read from the sensor into the ROS message.  When you run this node,  You will be publishing a [ROS Range message](http://docs.ros.org/api/sensor_msgs/html/msg/Range.html) which is a standard message included with ROS.

## Checkoff:
Using `rostopic echo /pidrone/tof` or the height graph on the web interface, verify that:

  * The TOF node is publishing a message with all of the fields you want
  * The range field of the message is a roughly accurate measure of the drone's altitude

You can now fly your drone with your own range node!
