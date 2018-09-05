# Project 2: Sensor Interfacing {#sensors-velocity status=draft}

## Velocity Estimation via Optical Flow
In this part of the project you will create a class that interfaces with the picamera to extract planar velocities from optical flow vectors. You will first write a class to analyze the sensor data, and then you will write a ROS node to instantiate the class and begin the data analysis.

## Camera Calibration
**Exercises**
  1. Describe the orientation of the camera relative to the drone using a [3d rotation matrix](https://en.wikipedia.org/wiki/Rotation_matrix)

## Analyze and Publish the Sensor Data
On your drones, the chip on the Raspberry Pi dedicated to video processing from the camera calculates motion vectors ([optical flow](https://en.wikipedia.org/wiki/Optical_flow)) automatically for H.264 video encoding. [Click here to learn more](https://www.raspberrypi.org/blog/vectors-from-coarse-motion-estimation/). You will be analyzing these motion vectors in order to estimate the velocity of your drone.

**Exercises**
You will now implement your velocity estimation using optical flow by completing all of the `TODO`'s in student_analyze_flow.py. There are two methods you will be implementing.

The first method is `setup`, which will be called to initialize the instance variables.
  1. Initialize the variables that you will use to scale the raw motion vectors from the camera
  2. Create a ROS publisher to publish the velocity values
  3. Create a ROS subscriber to obtain the altitude (z-position) of the drone for scaling the motion vecotrs

The second method is `analyze`, which is called every time that the camera gets an image, and is used to analyze the flow vectors to estimate the x and y velocities of your drone
  1. Estimate the velocities
  2. Publish the velocities


<!-- ## Create the ROS Node
In order for your utilize the methods you wrote on the camera data, an `AnalyzeFlow` object needs to be instantiated, and the `start_recording` method of the [PiCamera class](https://picamera.readthedocs.io/en/release-1.12/api_camera.html?highlight=start_recording) needs to be called with the `AnalyzeFlow` object as the `motion_output` parameter. This will redirect the camera output to your `AnalyzeFlow` object. Another arguement, `format`, must also be set to `h264` to extract the motion vectors from the formatting. In addition, the camera recording will also be used to extract position data, so the argument, `splitter_port` but be set equal to a port number, we'll use `1`. When you quit the script, the camera will need to `stop_recording` on that port. -->
