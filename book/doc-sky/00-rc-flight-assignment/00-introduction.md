# Flight Dynamics {#part:control_introduction status=ready}

Now that you've created a UKF to estimate your robot's state, you can
control its state using controller based on control theory. There are
many types of controllers that vary in levels of detail and
precision. A very simple controller is the PID controller, which you
will implement in the next project. Many other control algorithms
exist and can be applied to your drone using the data you have already
collected. In this sub-section, you will read about a few of the other
control options, and then you will going to fly an remote control (RC)
drone in simulation. Flying in simulation is quite realistic, and it
will give you a good feel for how difficult it is to control the drone
by hand. This activity will give you a greater understanding off how
much the control algorithm does to keep the drone flying steady.
