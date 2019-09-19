# Interfacing with the IMU {#sensors-assignment-imu status=ready}

Your drone is equipped with a [Skyline32 Flight Controller](https://hobbyking.com/en_us/skyline32-acro-flight-controller-w-baseflight-cleanflight.html) which has a built in IMU<sup id="a1">[1](#f1)</sup>. In this part of the project, you will learn how to interface with the flight controller board to extract the attitude, accelerations, angular rates of the drone from the built-in IMU. In addition, you will extract the battery levels from the flight controller so that you'll be able to tell when you're battery is too low.

**Setup**
Change directories into `~/ws/src/pidrone_pkg` and modify _pi.screenrc_ to start up with your flight controller node by changing `python flight_controller_node.py\n` to `rosrun project-sensors-yourGithubName student_flight_controller_node.py\n`. You can test your script by starting up screen and navigating to \`3 and using print statements.

## Understanding the Interface
Before extracting data from the flight controller, we must understand what data we can extract and how we extract it. The answer to the first question largely comes from the sensors that the flight controller contains, namely, the imu, and is addressed above. The next thing to look at is how to extract the data. In the problems below, you'll go through the process of getting all of this data. Before doing so, open up the file `h2rMultiWii.py` that is located in the `~/ws/src/pidrone_pkg/scripts` directory on your drone. The methods contained in this file are what we use to interface with the flight controller board through the USB. Do not worry about understanding each method; we do not even use them all. You will only need to understand the methods that we're about to walk through. Take a look at the `getData` method, this is what you'll use to get data from the board. Follow this method to the `receiveDataPacket` method. This is what extracts the data based on the code we give it (i.e. `MultiWii.ATTITUDE`). The three codes you'll be using in this project are `MultiWii.ATTITUDE`, `MultiWii.RAW_IMU`, and `MultiWii.ANALOG`. Take a look at which instance variables each of these methods modify. Now that you've been exposed to the interfacing protocol, let's work on getting the data!

## Problem 1: Extracting the Battery Data
The flight controller board is also capable of reading the voltage and current of the battery. This is useful information for us because it allows us to shut down the drone if the battery voltage is too low (Lipo batteries are quickly ruined if discharged too low).

** Exercises **

1. Take a look at Battery.msg in the ~/ws/src/pidrone_pkg/msg directory on your drone. This is a custom message we've created to communicate the battery values
2. Fill in the `TODO`s in the `update_battery_message` method in `student_flight_controller_node.py`.
3. Verify that the data is being published properly by running this node and echoing the topic ``/pidrone/battery`


## Problem 2: Extracting IMU data
Now we will extract linear accelerations and attitude (roll, pitch, yaw). In addition, we can discretely calculate the angular rates d/dt * (roll, pitch, yaw) by calculating the change in values (dr, dp, and dh) and dividing by the change in time since the previous values were stored (dt).

** Exercises **

1. Take a look at the [Imu ROS message type](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html) to get an understanding of the date you'll be collecting.
2. Uncomment the `update_imu_message` method in `student_flight_controller_node.py`and fill in all of the `TODO`s
3. Verify that the data is being published properly by running this node and echoing the topic ``/pidrone/imu`



###### Footnotes
[<b id="f1">1</b>](#a1) We will note now that you will also interface with the flight controller to control the motors of the drone. The flight controller is the component that controls the actuators (the motors) by sending PWM values to the ESCs. What all of these terms mean will be discussed later on when we focus on controlling and actuating the robot; for now, we are only focused on the the sensors on the flight controller.
