# Project Checkoff  {#sensors-assignment-checkoff status=ready}

## Functionality Check

1. Run `student_infrared_pub.py` and open up the web interface. Move the drone up and down and ensure that the height readings are reasonable.
2. Run `student_vision_flow_and_phase.py` and open up the web interface. Turn on position control. Slowly move the drone around over a highly textured planar surface and ensure that the raw position readings are reasonable.


## Questions

You will be asked to answer one of the following questions:

1. What types of measurements does the flight controller report in order to describe the orientation of the drone? What do we do to these measurements and why?
2. How does optical flow allow us to estimate the planar velocity of the drone? Why do we need to fly over a textured surface?
3. Why do we have a `state_callback` in `student_analyze_phase.py`? What do we do with the state information?
