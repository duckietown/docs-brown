# Project 2: Sensor Interfacing {#sensors-project status=ready}

## Overview
In this project, you will be interfacing with your drone's sensors to extract data, parse it into useful values, and publish the data on ROS topics. First, you will interface with the infrared range sensor, thus providing the drone with knowledge of its height relative to the ground. Then, you will interface with the IMU through the flight controller to extract the attitude of the drone (roll, pitch, and yaw), linear accelerations, and calculate the angular rates. Finally, you will interface with the camera to extract velocities using optical flow, and positions using rigid transforms. Woah, that's a lot of data! This is because you are in fact obtaining all of the information from each sensor that you will need for the drone to fly autonomously. In the next project, you will write a state estimator which fuses all of this sensor data to estimate the state of the drone.

## How this project fits into software stack
Take a look at the [software architecture diagram](https://docs-brown.duckietown.org/opmanual_sky/out/software_architecture_assignment.html) and notice the hardware components: <i>Flight Controller</i>, <i>Infrared Sensor</i>, and <i>Camera</i>. This is the hardware you'll be interfacing with in this project. Also notice the corresponding ROS nodes in the diagram. These are the ROS nodes you'll be creating to extract and publish sensor values.

## A note about how to approach this project
These docs give a high-level overview of the project. You will find more detailed directions in the stencil code. If you are unsure about what you have to do after reading these docs, the stencil code should give you a clearer idea.

## Handin
Use [this link](https://classroom.github.com/a/FvtacdCi) to generate a GitHub repo for this project. Clone the directory to your computer with `git clone https://github.com/h2r/project-sensors-yourGithubName.git`. This will create a new folder. The _README.md_ in your repo provides short descriptions of each project file.

When you submit your assignment, your folder should contain modified versions of the following files in addition to the other files that came with your repo:

* <i>student_infrared_pub.py</i>
* <i>student_analyze_flow.py</i>
* <i>student_analyze_phase.py</i>
* <i>student_flight_controller_node.py</i>

Commit and push your changes before the assignment is due. This will allow us to access the files you pushed to GitHub and grade them accordingly. If you commit and push after the assignment deadline, we will use your latest commit as your final submission, and you will be marked late.

```
cd project-sensors-yourGithubName
git add -A
git commit -a -m 'some commit message. maybe hand-in, maybe update'
git push
```

Note that assignments will be graded anonymously, so please don't put your name or any other identifying information on the files you hand in.
