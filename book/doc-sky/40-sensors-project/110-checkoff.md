# Project Checkoff  {#sensors-assignment-checkoff status=ready}

## Functionality Check

1. Run `student_tof_pub.py` and open up the web interface. Move the drone up and down and ensure that the height readings are reasonable.
2. Run `student_optical_flow_node.py` and `student_rigid_transform_node.py` and open up the web interface. Turn on velocity control (enabled by default). Slowly move the drone around over a highly textured planar surface and ensure that the raw velocity readings are reasonable.

<!-- this 2. is provisional since we have a bug where drones cannot be set to position mode. Currently working on fixing this, and then should be switched back -->


## Questions

You will be asked to answer one of the following questions:

1. What types of measurements does the flight controller report in order to describe the orientation of the drone? What do we do to these measurements and why?
2. How does optical flow allow us to estimate the planar velocity of the drone? Why do we need to fly over a textured surface?
3. Why do we have a `state_callback` in `student_analyze_phase.py`? What do we do with the state information?
