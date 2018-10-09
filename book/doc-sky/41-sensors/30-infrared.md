# Using your Infrared Range Finder {#sensors-assignment-infrared status=ready}

In this part of the project, you will learn how to estimate the drone's height using its infrared sensor. The drone is equipped with a [Sharp 2Y0A21YK0F Infrared Distance Sensor](https://www.digikey.com/product-detail/en/parallax-inc/28995/28995-ND/3523692), which is used for estimating the distance from the drone to the ground-plane. The sensor outputs an analog voltage signal (roughly 3.1V at

0.08 meters to 0.4V at 0.8 meters), which is converted to digital by our [Analog to digital Converter (ADC)](https://www.digikey.com/product-detail/en/adafruit-industries-llc/1083/1528-1014-ND/4990763) and read in by the Raspberry Pi as a 12-bit integer. The voltage value corresponds to distance, but we are going to need to do some work to convert it to real-world units.

**Setup**
Change directories into `~/ws/src/pidrone_pkg` and modify _pi.screenrc_ to start up with your infrared node by changing `python infrared_pub.py\n` to `rosrun project-sensors-yourGithubName student_infrared_pub.py\n`. You can test your script by starting up screen and navigating to \`7. You may stop _student_infrared_pub.py_ with ctrl-c, edit it within that tab, and then re-run `rosrun project-sensors-yourGithubName student_infrared_pub.py` to test your changes.

## Problem 1: Calibrate your IR Sensor
In _student_infrared_pub.py_, implement the method `calc_distance`, which takes in the 12-bit integer voltage from the ADC and calculates a distance **in meters**. Note that distance is **inversely proportional** to voltage:

$$d = 1/V$$

and you will need to both rescale and offset your distance:

$$d = m*(1/V) + b$$.

Every sensor is a little bit different (we estimate by as much as 10%), so in order to maximize the performance of your drone you will need to calibrate your sensor. To find your calibration parameters $m$ and $b$, you can measure the drone's height with a ruler or tape measure and print out the corresponding voltages. Take at least two measurements between 0.1m and 0.5m from the sensor; more measurements will yield a better estimate of the parameters.

## Problem 2: Publish your IR Reading
You are now collecting and processing sensor data! It's time to hook everything up to ROS so we can use those values elsewhere in the system.

In _student_infrared_pub.py_, implement the `publish_range` method and all of the "TODO"s in the `main` method. You will create a ROS node, and continuously use IR sensor readings to calculate and publish the drone's altitude. You will be publishing a [ROS Range message](http://docs.ros.org/api/sensor_msgs/html/msg/Range.html) which is a standard message included with ROS.

## Checkoff:
Using `rostopic echo /pidrone/infrared` or the IR graph on the web interface, verify that:

  * The IR node is publishing a message with all of the fields you want
  * The range field of the message is a roughly accurate measure of the drone's altitude

You can now fly your drone with your own infrared node!
