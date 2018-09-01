# Part 2: Tuning {#pid-assignment status=ready}

In this portion of the project, you will be tuning the planar PID controllers that we have implemented for you, and then transferring the altitude pid you created onto your drone. You will then tune the PID gains on your drone as you did in the simulator.

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

While flying, the low-rate I-terms will change to account for the static flight error, and when you disarm the drone, the initial low-rate I terms will be set to these changed values, thus allowing the low-rate I terms to start at this corrected value. Eventually, these values will converge, and your drone will no longer drift. Once converged, you will save the values by modify the variables `self.roll_low.init_i` and `self.pitch_low.init_i` in `pid_class.py` to the corresponding value printed in \`4 of the screen after disarming. This will store the initial low-rate I-terms between flights.  

  1. Perform one flight. After the drone takes off, do not give it movement commands but allow it to drift.  
  2. Disarm the drone before it flies too far in any direction, and move the drone back to the center of the flying area.  
  3. Note the values printed in \`4 of the screen.  
  4. Repeat steps 1-3 until the values that are printed out after disarming have converged (roughly when the change in magnitude is less than 1).  
  5. Once these values have converged, record these values in your answers  

## Problem 4: Flying with Your Altitude PID!
Now that the planar PIDs are tuned, and you have found a value for `throttle_low.init_i` that allows the drone to take off at a reasonable rate, you will be using your altitude PID to control the height of the drone. To tune your altitude PID, you will first use the Ziegler-Nichols tuning method to generate an initial set of tuning values. You will then fine tune these values similar to how you tuned the drone in simulation.  

To do this, we'll quit the <i>pid_controller.py</i> that runs in \`4 of the screen, and we'll instead run <i>student_pid_controller.py</i><sup id="a2">[2](#f2)</sup>. This will allow your PID to run alongside our planar PIDs, and on top of our throttle low-rate I-term which you found previously. Your PID will be responsible for keeping the drone flying steady vertically.  

**Exercises**
Change directories to `~/ws/src`. Run `git clone https://github.com/h2r/project3pid-yourGithubName.git`. Then change directories back to `~/ws/` and run `catkin_make --pkg project3pid-yourGitHubName`.

Prepare your drone to fly and then navigate to \`4 of the screen. Press ctrl-c to quit the pid_controller.  

In this screen, modify `~/ws/src/project3pid-yourGitHubName/z_pid.yaml` by setting $K$ to 1250 and the rest of the gain constants to 0. Now run `rosrun project3pid-yourGitHubName student_pid_controller.py` to fly with your altitude PID.  

Fly your drone and observe its flight. Tune $K_p$ by slowly increasing its value between flights until you can see the drone moving up and down with uniform oscillations. Each time you will need to quit the controller, edit `~/ws/src/project3pid-yourGitHubName/z_pid.yaml`, and then run `rosrun project3pid-yourGitHubName student_pid_controller.py` again to use the new PID terms.  

  1. Record your final $K_p$ value that causes uniform oscillations as $K_u$, the ultimate gain.  
  2. Fly your drone and pause the altitude graph on the web interface when you see two peaks. Find the time difference between these two peaks and record this value as $T_u$, the ultimate period.  
  3. Use your $K_u$ and $T_u$ values to compute $K_p$, $K_i$, and $K_d$. Refer to the equations in the Ziegler-Nichols section in the introduction to this project. Record these values and change <i>z_pid.yaml</i> accordingly.  
  4. Fly your drone with the set of tuning values generated by the Ziegler-Nichols method. Note that the Ziegler-Nichols method should enable safe flight, but will probably not control your drone's altitude very well! Empirically tune the gain constants in <i>z_pid.yaml</i> on your drone as you did in the simulator portion of this project. <sup id="a3">[3](#f3)</sup>  
  5. Report your final tuning values.  

Take a video of your drone flying first using our altitude pid by running <i>pid_controller.py</i> in \`4, then take a video of your tuned pid by running <i>student_pid_controller.py</i> in \`4. See if you can get yours to track the altitude setpoint better than ours! The drone should get to the setpoint quickly and stay there without bouncing up and down. *Submit these videos in Github Classroom as 'original_controller' and 'student_controller'*

## Problem 5: Position Control
As described in the introduction, when switching from velocity control to position control, a cascaded PID controller is used. The position controller forms the outer loop which provides setpoints for the inner loop velocity controller.  

Once you have achieved good trim and can fly steady with velocity control, you can try position control. Try to fly your drone for an entire battery without touching the controls! Do not try this until your I-term preloads have been tuned as described above.  

This [video](https://www.youtube.com/embed/WTohnsKs7dU) demonstrates the drone doing a zero velocity hover and drifting in the scene. Then we turn on position hold (you can tell when it is engaged when the drone's throttle drops) and it holds its position for several minutes.  

Then we turn off the position hold so you can see it drift again, and then turn it on again at the end and land. You can tell when it is turned on because we move the drone back to the center of the flight area before each hold.  

Engage position control in two steps. First you have to tell the drone to “remember” a frame. You can do this using the _r_ key. This will save the frame at the drone’s current location. Next you have to engage or disengage position control. You can engage this mode with the _p_ key, and disengage with _v_ for velocity control. So the procedure is to first save a frame (target location for the position hold) using _r_ and then shortly after (before drifting too much) type _p_.  

Position hold works best over a textured surface with lots of visual contrast. Even when doing position hold, always be ready to kill in case of a mishap. Especially be careful when looking at other windows.  

**Exercises**  
1. Engage position hold using the procedure described above. Observe the drone's behavior. How is it different from velocity mode?  
2. How long are you able to hold position? Ideally you should be able to do this in one spot for an entire battery. If not, try re-tuning your I-term preloads above. If you're flying on the power supply instead of a battery, the drone should stay in place indefinitely, but you can stop it after 5 minutes.  
3. Turn off position hold with _v_ and make sure there is room to fly to the right. Press and hold 'L' and observe the drone's motion, and release 'L' to stop the drone from moving. Explain what happens to the following key terms that causes the drone to move when you press 'L' and stop when you release: setpoint, error, control variable, process variable, proportional term, integral term, derivative term. We are looking only for a higher level description to demonstrate understanding of the PID controllers.  
4. While flying in position control, make sure there is room for the drone to fly to the right and then take note of the desired position in \`4 of the screen. Now press the 'L' key in the user interface and note the new desired x-position of the drone; it should be 0.1m to the right of the drone's last position. Explain what happens to the following key terms of the outer control loop, the position PID, that causes the drone to move and stop 0.1m to the right after you press 'L' in position control: setpoint, error, control variable, process variable, proportional term, integral term, derivative term. We are looking only for a higher level description to demonstrate understanding of cascaded PID controllers.  
5. When the 'L' is first pressed, the derivative term spikes. Briefly explain why the derivative term spikes, and one possible way to prevent this spike.  

Take a one minute video of your drone flying in velocity control, and then engage position control. *Submit this video in Github Classroom as 'postion_control'*  

***Optional Exercises (not graded, but cool to try!)***  
6. Try flying in velocity mode over a blank white poster board (there is one in 121). Be careful! How does the drone's behavior change?  
7. Now try over a uniform textured surface such as the floor in 121. Try a position hold. How well does it work? How long is it able to hold position?  

###### Footnotes
[<b id="f1">1</b>](#a1) It is important to do these tests over a textured surface so that the planar motion of the drone can be properly sensed using the optical flow vectors of the camera (more on this in another section)

[<b id="f2">2</b>](#a2) You can make your future life easier by modifying _pi.screenrc_ to always run your script by changing `python pid_controller.py\n` to `python student_pid_controller.py\n`

[<b id="f3">3</b>](#a3) Try to just focus on the the altitude while doing this and ignore the planar motion because it is easiest to focus on one axis at a time when tuning the PIDs. The planar axes can be re-tuned after you tune your altitude pid if need be. It can be extremely useful to use graph on the web interface to visualize the drone's ability to hover at the setpoint which starts as 0.3m
