# Assignment {#ros-assignment status=ready}

First set up the docker image following [these instructions](docker.html).

## Creating a Publisher and Subscriber (50 points)



*Answer these questions in `ros.txt`  and submit the ROS package you create.*

1. Read [understanding nodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes).
2. Start the `screen` session we use to fly the drone. Use `rosnode list` to display what nodes are running when you start the screen. If you wish, take a look at the [software architecture diagram](https://docs-brown.duckietown.org/opmanual_sky/out/software_architecture_assignment.html#sec:software-architecture-assignment) and look at all of the blue ROS topics to gain a visual understanding of all of the nodes that are running. Once again, do not worry about understanding everything now, or knowing what each topic is used for- you will learn this through experience as the course progresses. *No answer is required for this question*
3. Use `rosnode info` to find out more about as many nodes as you'd like. What is the name of the time of flight (tof) node and what topics does it publish?
4. Do the ROS tutorial to [create a package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage). Name your package `ros_assignment_pkg`.  Create it in the `catkin_ws/src` directory on your drone. 
5. Do the [building packages](http://wiki.ros.org/ROS/Tutorials/BuildingPackages) tutorial.

6. Follow the [ROS publisher/subscriber tutorial](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29) using the workspace and package you created above. *Hand in the entire package.*
7. Start the `screen` session we use to fly the drone. Use `rostopic echo` and `rostopic hz` to examine the results of various topics. What is the rate at which we are publishing the infrared range reading?

## Messages (5 points)

*Make all modifications in your ROS package from Problem 1 and hand in the package*

1. Read [Creating a ROS msg](https://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv). You do not need to read the section on services.
2. In your package from question 1, create a ROS message called `MyMessage`
   with a field for a `string`, called `name`, and a field for an array of
   `float64`, called `contents`. Edit files such as `CMakeLists.txt` to ensure
   your message is compiled and available for use. *Make these modifications in the package from problem 1 and hand it in.*

## Reading the TOF Sensor (15 points)

1. Write a ROS subscriber on your drone to read the values from the time of flight
   sensor topic and print them to `stdout`. *Name the file `my_echo.py` and
   submit it.*
2. Write a second ROS subscriber that listens to the time of flight sensor topic and
   calculates the mean and variance over a ten second window using
   [NumPy](https://jakevdp.github.io/PythonDataScienceHandbook/02.02-the-basics-of-numpy-arrays.html). Print these values to `stdout`. *Name the file `mean_and_variance.py` and submit it.*

## Handin

When you are done, use [this link](https://classroom.github.com/a/aOpoHGtd) to create your assignment Github Repo.

- `my_echo.py`, `mean_and_variance.py`
- `ros.txt`
- `ros_assignment_pkg`
