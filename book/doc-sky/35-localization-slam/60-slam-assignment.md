# SLAM Assignment {#localization-slam-assignment status=ready}

The assignment is very similar to the localization assignment. You will be given four files:

    onboard_slam.py
    slam_helper.py
    utils.py
    thread_queue.py

onboard_slam, utils, and thread_queue consist of fully implemented helper code that you are welcome to read but do not need to edit. You may, however, edit NUM_PARTICLE and NUM_FEAUTURE in onboard_slam.py to experiment with the speed/accuracy tradeoff of the number of particles in the filter and the number of features extracted in each frame by OpenCV.

Note the two files we have not yet mentioned, utils and thread_queue. utils contains add_landmark and update_landmark as promised, as well as some other helper code. thread_queue implements a class which uses Python threads to help manage map updates. As you have likely noticed, the map_update step in FastSLAM is much more involved than the measurement update in MC Localization. Since it is so slow, it is not acceptable to perform motion prediction and map updates in the same thread. The drone may move during the time it takes the map to update, and the motion prediction will not be able to account for this movement! Instead, you will spawn a thread to take care of the map updates in parallel<sup>*</sup> to the motion prediction.

The job of the ThreadQueue class is straightforward: ensure there is at most one map update thread running at a time. If you add a thread to an empty queue, start the thread. If you try to add a thread to the queue and one is already is progress, throw away the new thread.

Your assignment is to implement the missing parts of the slam_helper file, which are indicated with TODOs. The intended functionality of each missing method is indicated in the docstrings.

<sup>*</sup>For those interested, your Python threads spawned with the threading module will actually not run in parallel.

## Testing
Testing your SLAM implementation will work similarly to testing your localization code. You can run vision_localization_onboard.py with the flag "--SLAM" to test your code, as long as your slam_helper file in present in the scripts folder. You should check to see that the poses printed to the terminal roughly match the movements of your drone (if your drone moves to the right, the x pose increases). You may also use animate_particle_filter.py as described above to test the code. Please **do not** expect this to work well, even if your implementation is perfect. Read on  to find out why.

**Computational Constraints**
Unfortunately, our raspberry pi's are not powerful enough to run FastSLAM and their own PID controllers, etc in real time. Trying to do so will result in a drone flying haywire and SLAM that is too slow to produce accurate poses. We have a couple options to work around this problem:

 1. Network in an offboard computer computer to run your SLAM programs while the drone is flying. This option is ideal, but only feasible if you have access to a computer running Ubuntu 14.04 with ROS kinetic installed.
 2. Sequentially perform SLAM and localization. This involves flying the drone over the area you want to map and recording the data received by the camera (keypoints and descriptors) during the flight. The drone then lands and runs the collected data through FastSLAM to build a map. Finally, the drone flies and runs a modified version of localization which uses the map produced by SLAM to localize in real time.

We recommend that you develop your project in the following way: implement SLAM and debug while testing the code running on your drone in real time (onboard online). Once you feel you have a working implementation, you should follow the directions in PiDrone Vision Instructions to test your SLAM using option 2 above (onboard offline). You are also welcome to use an ubuntu to test your implementation in real time (offboard online) following the directions in PiDrone Vision Instructions. We ask that you complete the checkoff below before using one of our offboard computers to test.
