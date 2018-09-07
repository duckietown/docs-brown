# Project 2: Sensor Interfacing {#sensors-velocity status=draft}

## Position Estimation via OpenCV's estimateRigidTransform
In this part of the project you will create a class that interfaces with the picamera to extract planar positions of the drone relative to the first image taken using OpenCV's estimateRigidTransform function.

## Code Structure
By now, you've become a pro at extracting sensor data, so you have two options for this portion of the project: you can either create a blank file and save it as `student_analyze_phase.py` to write your AnalyzePhase class from scratch, or you can use the existing `student_analyze_phase.py` and the included stencil code. The choice is up to you, but we will say that there is something quite rewarding about creating a complete script from scratch. If you choose to use the stencil code, skip steps 1-4 below.

**Exercises**
1. Create a new file, `student_analyze_flow.py` in your sensors project repository.
2. In this file, create a class `AnalyzePhase` which inherits from `picamera.array.PiMotionAnalysis`
3. Your AnalyzePhase class needs to implement the functions setup(self, camera_wh) where camera_wh is a tuple of the video dimensions and write(self, data) where data is the next frame getting passed in from the video stream. Setup will be called when the class is instantiated, and write will be called each time we get a new frame from the video stream.
4. At the top of the picam_pos_class.py add the following imports:
  ```
  import tf
  import cv2
  import rospy
  import picamera
  import numpy as np
  from pidrone_pkg.msg import State
  from std_msgs.msg import Empty, Bool
  from geometry_msgs.msg import PoseStamped
  ```
5. Once your class is setup correctly, you should be able to run `student_vision_flow_and_phase.py` in \`6, and if you print the `data` argument in the write function in `AnalyzePhase`, you should see new image frames getting passed in.

## Analyze and Publish the Sensor Data
To estimate our position we will make use of OpenCV’s [<i>estimateRigidTransform</i>](https://docs.opencv.org/3.0-beta/modules/video/doc/motion_analysis_and_object_tracking.html#estimaterigidtransform) function. This will return an affine transformation between two images if the two images have enough in common to be matched, otherwise, it will return None.

**Exercises**
The first method you'll complete is `setup`, which will be called to initialize the instance variables.
  1. Initialize the variables that you will use to scale the raw motion vectors from the camera
  2. Create a ROS publisher to publish the velocity values
  3. Create a ROS subscriber to obtain the altitude (z-position) of the drone for scaling the motion vecotrs

The second method is `analyze`, which is called every time that the camera gets an image, and is used to analyze the flow vectors to estimate the x and y velocities of your drone
  1. We’ll need to convert our image data into a more usable format:
    `np.reshape(np.fromstring(data, dtype=np.uint8), (240, 320, 3))``
  2. Save the first image and then compare subsequent images to it using cv2.estimateRigidTransform. (Note that the fullAffine argument should be set to False. Can you see why?)
  3. If you print the output from estimateRigidTransform, you’ll see a 2x3 matrix when the camera sees what it saw in the first frame, and a None when it fails to match. This 2x3 matrix is an affine transform which maps pixel coordinates in the first image to pixel coordinates in the second image. Read [this article](https://picamera.readthedocs.io/en/release-1.10/api_array.html#pimotionanalysis) for details on how to extract useful information from this affine transform.
  4. Implement a method in your `AnalyzePhase` class which takes an affine transform and returns the x and y translations of the camera and the yaw. This is tricky!!! You will need to make use of camera_wh to normalize from pixel coordinates.
  5. As with velocity measurements, the magnitude of this translation in global coordinates is dependent on the height of the drone. Add a subscriber to the topic /pidrone/state and save the value to a class variable in the callback. Use this variable to compensate for the height of the camera in your method from step four which interprets your affineTransform

## Integrate Position to Increase your Functional Area
Simply matching against the first frame is not quite sufficient for estimating position because as soon as the drone stops seeing the first frame it will be lost. Fortunately we have a fairly simple fix for this: compare the current frame with the previous frame to a get “position step”, and integrate these position steps to maintain a position estimate when not seeing the first frame. The framerate is high enough and the drone moves slow enough that the we will almost never fail to match on the previous frame.

**Exercises**
Modify your AnalzePhase class to add the functionality described above.
1. Store the previous frame. When estimateRigidTransform fails to match on the first frame, run estimateRigidTransform on the previous frame and the current frame.
2. Create a class variable to store the position (x,y,yaw) of the drone and initialize it to the origin in setup. When you fail to match on the first frame, integrate your “position steps” in this position variable.
3. Modify your state_callback method to update the class variables: x, y, yaw, with the current values of the drone

**Note** The naive implementation simply sets the position of the drone when we see the first frame, and integrate it when we don’t. What happens when we haven’t seen the first frame in a while so we’ve been integrating, and then we see the first frame again? There may be some disagreement between our integrated position and the one we find from matching with our first frame due to accumulated error in the integral, so simply setting the position would cause a jump in our position estimate. The drone itself didn’t actually jump, just our estimate, so this will wreak havoc on whatever control algorithm we write based on our positition estimate. To mitigate these jumps, you should use a filter to blend your integrated estimate and your new first-frame estimate using a filter. Since this project is only focussed on publishing the measurements, worrying about these discrepencies is unnecessary. In the next project, you will be implementing a UKF to use these measurements to accurately estimate the state of the drone.

## Connect to the JavaScript Interface
Now that we’ve got a position estimate, let’s begin hooking our code up to the rest of the flight-stack.
  1. Create a subscriber (in the setup function) to the topic `/pidrone/reset_transform` and a callback owned by the class to handle messages. [ROS Empty messages](http://docs.ros.org/lunar/api/std_msgs/html/msg/Empty.html) are published on this topic when the user presses r for reset on the JavaScript interface. When you receive a reset message, you should take a new first frame, and set your position estimate to the origin again.
  2. Create a subscriber to the topic `/pidrone/position_control`. [ROS Bool messages](http://docs.ros.org/lunar/api/std_msgs/html/msg/Bool.html) are published on this topic when the user presses `p` or `v` on the JavaScript interface. When we’re not doing position hold we don’t need to be running this resource-intensive computer vision, so when you receive a message you should enable or disable your position estimation code.

## Checkoff 1
Print out your position estimate in the write function and submit a video entitled, "anaylze_phase", of the following tasks in order to be checked off:
* Initialize and reset your first frame matching from the JavaScript Interface
* Toggle your position estimation from the JavaScript interface
* Find the first frame repeatedly
* Integrate frame-by-frame error when your drone cannot see the first frame
* Recognize your first frame after integrating for a few seconds and smoothly blend the position back together (print out the * delta-position for this part)

## Measurement Visualization
Debugging position measurments can also be made easier through the use of a visualizer. A few things to look for are sign of the position, magnitude of the postition, and the position staying steady when the drone isn't moving. Note again that these measurements are unfiltered and will thus be noisy; don't be alarmed if the position jumps when it goes from not seeing the first frame to seeing it again.

**Exercises**
Use the rqt_plot package to visualize your x and y position estimates
1. Create a file, 'position_visualizer.py' in your sensors project repository.
2. Connect to your drone using `ssh -Y pi@yourhostname` so that you can use graphical clients in your ssh session
3. In any window of the screen (except \`0 which is roscore), quit the current program that's running, and run `rqt_plot /pidrone/picamera/pose/x:y` to subscribe to topic `/pidrone/picamera/pose` and plot the x and y positions

## Checkoff 2
1. Verify that the position values are reasonable (roughly in the range of -1m to 1m) and have the correct sign (positive when the drone is moving to the right or up, and negative to the left or down).
2. Submit a video, `x_position_visualization` of your visualizer running while moving your drone left and right when the first frame is in view and when it is not
3. Submit a video, `y_position_visualization` of your visualizer running while moving your drone forward and backward when the first frame is in view and when it is not
