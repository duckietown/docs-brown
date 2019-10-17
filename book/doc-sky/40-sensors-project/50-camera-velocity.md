# Velocity Estimation via Optical Flow {#sensors-velocity status=ready}

In this part of the project you will create a class that interfaces with the Arducam to extract planar velocities from optical flow vectors.

## Code Structure
To interface with the camera, you will be using the picamera library. This library allows you to use classes which inherit from [<i>picamera.array.PiMotionAnalysis</i>](https://picamera.readthedocs.io/en/release-1.10/api_array.html#pimotionanalysis) to receive and analyze frames from the video stream. In the sensors project repo, we've include a script called student_vision_flow_and_phase.py which instantiates objects of your analyze classes that inherit <i>picamera.array.PiMotionAnalysis</i> to allow the objects to receive and analyze frames from the video stream. This script creates what we call a <i>vision</i> node which is a ROS node we created that provides data from the camera. This node is called <i>vision_flow_and_phase</i> because it uses the two classes you'll be creating to analyze the camera data and provide velocity and position estimates. Later on in the course, you'll be creating a <i>vision_localization</i> node that uses localization to analyze the camera data and provide position estimates.

## Analyze and Publish the Sensor Data
On your drones, the chip on the Raspberry Pi dedicated to video processing from the camera calculates motion vectors ([optical flow](https://en.wikipedia.org/wiki/Optical_flow)) automatically for H.264 video encoding. [Click here to learn more](https://www.raspberrypi.org/blog/vectors-from-coarse-motion-estimation/). You will be analyzing these motion vectors in order to estimate the velocity of your drone.

**Exercises**

You will now implement your velocity estimation using optical flow by completing all of the `TODO`'s in student_analyze_flow.py. There are two methods you will be implementing.

The first method is `setup`, which will be called to initialize the instance variables.

  1. Create a ROS publisher to publish the velocity values

The perspicacious roboticist may have noticed that magnitude of the velocity in global coordinates is dependent on the height of the drone. Add a subscriber to the topic /pidrone/state to your AnalyzeFlow class and save the z position value to a class variable in the callback. Use this variable to scale the velocity measurements by the height of the drone (the distance the camera is from what it is perceiving).

  2. Create a ROS subscriber to obtain the altitude (z-position) of the drone for scaling the motion vectors

The second method is `analyze`, which is called every time that the camera gets an image, and is used to analyze the flow vectors to estimate the x and y velocities of your drone
  1. Estimate the velocities, using the `TODO`'s as a guide
  2. Publish the velocities

## Check your Measurements
You'll want to make sure that the values you're publishing make sense. To do this, you'll be echoing the values that you're publishing and empirically verifying that they are reasonable.

**Exercises**

Verify your velocity measurements

1. Start up your drone and launch a screen
2. Navigate to \`4 and quit the node that is running
3. Run `rosrun project-sensors-yourGithubName student_analyze_flow.py`
4. Enter `rostopic echo /pidrone/picamera/twist`
5. Move the drone by hand to the left and right and forward and backward to verify that the measurements make sense

## Checkoff
1. Verify that the velocity values are reasonable (roughly in the range of -1m/s to 1m/s) and have the correct sign (positive when the drone is moving to the right or up, and negative to the left or down).
