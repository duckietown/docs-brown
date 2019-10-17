# Position Estimation via OpenCV's estimateRigidTransform {#sensors-position status=ready}

In this part of the project you will create a class that interfaces with the picamera to extract planar positions of the drone relative to the first image taken using OpenCV's estimateRigidTransform function.

## Ensure images are being passed into the analyzer
Before attempting to analyze the images, we should first check that the images are being properly passed into the analyze method

**Exercises**

1. Open `student_analyze_phase.py` and print the `data` argument in the method `write`.
2. Navigate to \`4 and run `rosrun project-sensors-yourGithubName student_vision_flow_and_phase.py`. Verify that the images are being passed in by checking that values are printing out from where you told it to print `data`.

## Analyze and Publish the Sensor Data
To estimate our position we will make use of OpenCV’s [<i>estimateRigidTransform</i>](https://docs.opencv.org/3.0-beta/modules/video/doc/motion_analysis_and_object_tracking.html#estimaterigidtransform) function. This will return an affine transformation between two images if the two images have enough in common to be matched, otherwise, it will return None.

**Exercises**

The first method you'll complete is `setup`, which will be called to initialize the instance variables.

  1. Fill in all of the `TODO`s in `setup`

The second method is `write`, which is called every time that the camera gets an image, and is used to analyze two images to estimate the x and y translations of your drone.

  2. Save the first image and then compare subsequent images to it using cv2.estimateRigidTransform. (Note that the fullAffine argument should be set to False.)
  3. If you print the output from estimateRigidTransform, you’ll see a 2x3 matrix when the camera sees what it saw in the first frame, and a None when it fails to match. This 2x3 matrix is an affine transform which maps pixel coordinates in the first image to pixel coordinates in the second image. 
  4. Implement the method `translation_and_yaw`, which takes an affine transform and returns the x and y translations of the camera and the yaw.
  5. As with velocity measurements, the magnitude of this translation in global coordinates is dependent on the height of the drone. Add a subscriber to the topic /pidrone/state and save the value to `self.altitude` in the callback. Use this variable to compensate for the height of the camera in your method from step 4 which interprets your affineTransform.

## Account for the case in which the first frame is not found
Simply matching against the first frame is not quite sufficient for estimating position because as soon as the drone stops seeing the first frame it will be lost. Fortunately we have a fairly simple fix for this: compare the current frame with the previous frame to get the displacement, and add the displacement to the position the drone was in in the previous frame. The framerate is high enough and the drone moves slow enough that the we will almost never fail to match on the previous frame.

**Exercises**

Modify your AnalyzePhase class to add the functionality described above.

1. Store the previous frame. When estimateRigidTransform fails to match on the first frame, run estimateRigidTransform on the previous frame and the current frame.
2. When you fail to match on the first frame, add the displacement to the position in the previous frame. You should use `self.x_position_from_state` and `self.y_position_from_state` (the position taken from the `pidrone/state` topic) as the previous coordinates.

**Note** The naive implementation simply sets the position of the drone when we see the first frame, and integrates it when we don’t. What happens when we haven’t seen the first frame in a while so we’ve been integrating, and then we see the first frame again? There may be some disagreement between our integrated position and the one we find from matching with our first frame due to accumulated error in the integral, so simply setting the position would cause a jump in our position estimate. The drone itself didn’t actually jump, just our estimate, so this will wreak havoc on whatever control algorithm we write based on our position estimate. To mitigate these jumps, you should use a filter to blend your integrated estimate and your new first-frame estimate. Since this project is only focused on publishing the measurements, worrying about these discrepancies is unnecessary. In the UKF project, you will address this problem.

## Connect to the JavaScript Interface
Now that we’ve got a position estimate, let’s begin hooking our code up to the rest of the flight stack.

  1. Create a subscriber (in the setup function) to the topic `/pidrone/reset_transform` and a callback owned by the class to handle messages. [ROS Empty messages](http://docs.ros.org/lunar/api/std_msgs/html/msg/Empty.html) are published on this topic when the user presses r for reset on the JavaScript interface. When you receive a reset message, you should take a new first frame, and set your position estimate to the origin again.
  2. Create a subscriber to the topic `/pidrone/position_control`. [ROS Bool messages](http://docs.ros.org/lunar/api/std_msgs/html/msg/Bool.html) are published on this topic when the user presses `p` or `v` on the JavaScript interface. When we’re not doing position hold we don’t need to be running this resource-intensive computer vision, so when you receive a message you should enable or disable your position estimation code.

## Measurement Visualization
Debugging position measurements can also be made easier through the use of a visualizer. A few things to look for are sign of the position, magnitude of the position, and the position staying steady when the drone isn't moving. Note again that these measurements are unfiltered and will thus be noisy; don't be alarmed if the position jumps when it goes from not seeing the first frame to seeing it again.

**Exercises**

Use the web interface to visualize your position estimates

1. Connect to your drone and start a new screen
2. Run `rosrun project-sensors-yourGithubName student_vision_flow_and_phase.py` in \`4.
3. Hold your drone up about .25m with your hand
4. In the web interface, press `r` and the `p` to engage position hold.
5. Use `rostopic echo /pidrone/picamera/pose` to view the output of your <i>student_analyze_phase</i> class
6. Move your drone around by hand to verify that the values make sense.
7. Look at the web interface and see if it tracks your drone. Pressing `r` should set the drone drone visualizer back to the origin.

## Checkoff 
1. Verify that the position values are reasonable (roughly in the range of -1m to 1m) and have the correct sign (positive when the drone is moving to the right or up, and negative to the left or down).
