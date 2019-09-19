# Part 2: Planar Tuning {#pid-planar status=ready}

In this portion of the project, you will be tuning the low rate integral terms of the PID controllers that we've provided.

## Trimming your Drone
Due to differences in the weight distribution and other factors that cause asymmetries, the drone will tend to initially drift in a particular direction. In order to tune your altitude PID, the planar motion of the drone needs to be controlled. This is important so that the drone does not fly uncontrollably across the room while you're trying to tune its altitude controller. To control the drone's planar motion while you're tuning the altitude, we've created and tuned PIDs to do this for you, but you will need to tune the initial low-rate integral terms to account for the uneven weight distribution specific to your drone. You will first use the provided altitude PID to tune the planar controllers, and then you will tune your altitude PID with the tuned planar controllers.

Write brief answers to all exercises in <i>answers_pid.md</i>.

## Problem 1: Understanding the Controller
Our controller is a dual I-term (integral term) PID controller. The high-rate I-term changes quickly, allowing fast response to changes. The low-rate I-term changes slowly, allowing the drone to adjust to systemic sources of error. The provided PID gains have been pretuned to this drone hardware, and should not need significant modification for your specific drone. But, the initial low I-terms do need to be adjusted based on the static error of your specific drone.  

**Exercises**  
  1. Name a source of static error that the low-rate I term can correct for.  
  2. Name two sources of dynamic error that the high-rate I term can correct for.  

## Problem 2: Tune the Throttle
The first step in the tuning process is finding an initial throttle value that allows your drone to have a smooth and controlled takeoff. To do this, you'll be adjusting the value of `throttle_low.init_i` in <i>pid_class.py</i>. This is the initial value of the low-rate (slow changing) integral term for the throttle, which controls altitude. The default value is 100. you will tune this value by having the drone take off, observing its behavior, and modifying the value accordingly. Each time you wish to change the value, you will need to restart <i>pid_controller.py</i> to use the new value.  

**Setup**  
1. Prepare your drone to fly over a highly textured planar surface<sup id="a3">[1](#f3)</sup>.  
2. Navigate to \`4 of the screen.  
3. Quit the program by pressing ctrl-c.  
**Exercises**  
  1. In this screen (\`4), use a text editor (such as vim or nano) to modify `throttle_low.init_i` in <i>pid_class.py</i> to test out different values for `throttle_low.init_i`. Be cautious when modifying this value because the drone could take off abruptly with a value that is too high. The specific `throttle_low.init_i` value is drone specific, but typical values range between 50 and 150. Try both of these values and two more values between then. In one sentence, describe the drone's behavior as a result of changing the value up and down.  
  2. Now find the value for which your drone is able to have a smooth and controlled takeoff. The goal is to reduce the overshoot and undershoot for the drone to takeoff and fly stable at 0.3m. Try changing this value in increments of 10 and then 5 until you find a  value that allows the drone to take off at a reasonable rate. Record this value in your answers.  

## Problem 3: Set the Trim
Next you will set the trim on roll and pitch. You will do this by tuning the low I-terms to adjust for the static errors that exist on your drone. The default value is 0, and positive values will move the drone to the right or forward, and negative to the left or backward, depending on the axis you're modifying. Note that you may need to repeat this process periodically, for example after a crash or the like.
When performing this process, each time make sure that you:  

  * Place the battery in the same place each time as much as possible so the weight is distributed the same.  
  * Plug the flight controller while the struts are fully engaged and the drone is level, so the gyros are well calibrated.  
  * Always place the drone so that the camera is closer to you and the skyline is farther away.  

**Setup**  
Modify <i>pid_controller.py</i> to print out the low rate integral terms of the PIDs by finding the block of code shown below and uncommenting the following print statements

```
print 'roll_low.init_i', pid_controller.pid.roll_low.init_i
print 'pitch_low.init_i', pid_controller.pid.pitch_low.init_i
```

You will also need to set the `verbose` variable in this file to zero so that these print statements will not be overridden by the other print statements: `verbose = 0`

While flying, the low-rate I-terms will change to account for the static flight error, and when you disarm the drone, the initial low-rate I terms will be set to these changed values, thus allowing the low-rate I terms to start at this corrected value. Eventually, these values will converge, and your drone will no longer drift. Once converged, you will save the values by modifying the variables `self.roll_low.init_i` and `self.pitch_low.init_i` in `pid_class.py` to the corresponding value printed in \`4 of the screen after disarming. This will store the initial low-rate I-terms between flights.  

**Exercises**  

  1. Perform one flight. After the drone takes off, do not give it movement commands but allow it to drift.  
  2. Disarm the drone before it flies too far in any direction.  
  3. Write down the low-I values printed in \`4 of the screen.  
  4. Pick up and move the drone by hand back to the center of the flying area.  
  5. Repeat steps 1-4 until the values that are printed out after disarming have converged (roughly when the change in magnitude is less than 1).  
  6. Once these values have converged, record these values in your answers.  



###### Footnotes
[<b id="f3">1</b>](#a3)A flat posterboard scribbled or written on with marker will work. 
