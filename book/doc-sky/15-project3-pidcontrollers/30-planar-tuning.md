# Part 2: Planar Tuning {#pid-assignment status=ready}

In this portion of the project, you will be tuning the planar PID controllers that we have implemented for you. 

## Trimming your Drone
Each drone is built a little differently. Due to differences in the weight distribution and other factors, the drone will tend to initially drift in a particular direction.  

In order to tune the altitude PID, the planar motion of the drone needs to be controlled. This is important so that the drone does not fly uncontrollably across the room while you're trying to tune its altitude controller. To control the drone's planar motion while you're tuning the altitude, we've created and tuned PIDs to do this for you, but you will need to tune the initial low-rate I-terms to account for the uneven weight distribution specific to your drone. You will use our altitude PID to tune the planar controllers, and then you will tune your altitude PID with the tuned planar controllers.  

Write brief answers to all exercises in <i>answers_pid.md</i>.  

## Problem 1: Understanding the Controller
Our controller is a dual I-term PID controller. The high-rate I-term changes quickly, allowing fast response to changes. The low-rate I-term changes slowly, allowing the drone to adjust to systemic sources of error such as poor weight distribution or a damaged propeller. The PIDs have been tuned by us and should not need significant modification for your specific drone, but the initial low I-terms do need to be adjusted based on your specific drone.  

**Exercises**  
  1. Name a source of static error that the low-rate I term can correct for.  
  2. Name two sources of dynamic error that the high-rate I term can correct for.  

## Problem 2: Tune the Throttle
The first step in the tuning process is finding an initial throttle value that allows your drone to have a smooth and controlled takeoff. To do this, you'll be adjusting the `throttle_low.init_i` in <i>pid_class.py</i>. This is the initial value of the low-rate (slow changing) integral term for the throttle, which controls altitude. The default value is 100.  

**Exercises**  
Prepare your drone to fly over a highly textured planar surface<sup id="a1">[1](#f1)</sup>.  
  1. Use a text editor (such as vim or nano) to modify `throttle_low.init_i` in <i>pid_class.py</i> and test out different values for `throttle_low.init_i`. Be cautious because the drone could take off abruptly with a value that is too high. This is drone dependent, but typical values range between 50 and 150. Try both of these values and two more values between then. In one sentence, describe the drone's behavior as a result of changing the value up and down.  
  2. Now find and report the value for which your drone is able to have a smooth and controlled takeoff. The goal is to reduce the overshoot and undershoot for the drone to takeoff and fly stable at 0.3m. Try changing this value in increments of 10 and then 5 until you find a  value that allows the drone to take off at a reasonable rate.  

## Problem 3: Set the Trim
Next you will set the trim on roll and pitch. You will do this by tuning the low I-terms to adjust for the static errors that exist on your drone. The default value is 0, and positive values will move the drone to the right or forward, and negative to the left or backward, depending on the axis you're modifying. Note that you may need to repeat this process periodically, for example after a crash or the like.
When performing this process, each time make sure that you:  

  * Place the battery in the same place each time as much as possible so the weight is distributed the same.  
  * Tape or velcro the battery so it does not move.  
  * Plug the flight controller while the struts are fully engaged and the drone is level, so the gyros are well calibrated.  
  * Always place the drone so that the camera is closer to you and the skyline is farther away.  

**Exercises**  
Modify <i>pid_controller.py</i> to print out the low and high rate integral terms of the roll and pitch PIDs by finding the block of code shown below and uncommenting the print statements.  

```
# print 'roll_low.init_i', pid_controller.pid.roll_low.init_i
# print 'pitch_low.init_i', pid_controller.pid.pitch_low.init_i
# print 'throttle_low.init_i', pid_controller.pid.throttle_low.init_i
```

You will also need to set the `verbose` variable to zero so that these print statements will not be overridden by the other print statements: `verbose = 0`  

While flying, the low-rate I-terms will change to account for the static flight error, and when you disarm the drone, the initial low-rate I terms will be set to these changed values, thus allowing the low-rate I terms to start at this corrected value. Eventually, these values will converge, and your drone will no longer drift. Once converged, you will save the values by modifying the variables `self.roll_low.init_i` and `self.pitch_low.init_i` in `pid_class.py` to the corresponding value printed in \`4 of the screen after disarming. This will store the initial low-rate I-terms between flights.  

  1. Perform one flight. After the drone takes off, do not give it movement commands but allow it to drift.  
  2. Disarm the drone before it flies too far in any direction, and move the drone back to the center of the flying area.  
  3. Note the values printed in \`4 of the screen.  
  4. Repeat steps 1-3 until the values that are printed out after disarming have converged (roughly when the change in magnitude is less than 1).  
  5. Once these values have converged, record these values in your answers  
