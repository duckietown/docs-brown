# Tuning
In this portion of the project, you will be tuning the planar pid controllers that we have implemented for you, and then transferring the altitude pid you created onto your drone and will be tuning your pid gains on your drone as you did in the simulator.

## Trimming your Drone
Each drone is built a little differently. Due to differences in the weight distribution and other factors, the drone will tend to initially drift in a particular direction.

In order to tune the altitude pid, the planar motion of the drone needs to be controlled. This is important so that the drone does not fly uncontrollably across the room while you're trying to tune it's altitude controller. To control the drone's planar motion while you're tuning the altitude, we've created and tuned pids to do this for you, but you will need to tune the initial low-rate i-terms to account for the uneven weight distribution specific to your drone. You will use our altitude pid to tune the planar controllers, and then you will tune your altitude pid with the tuned planar controllers.

## Problem 1: Understanding the Controller
Our controller is a dual I term PID controller. The high-rate I term changes quickly, allowing fast response to changes. The low-rate I term changes slowly, allowing the drone to adjust to systemic sources of error such as poor weight distribution or a damaged propeller. The pids have been tuned by us and should not need significant modification for your specific drone, but the initial low i terms do need to be adjusted based on your specific drone. In the next part of the course we will go into more detail with this structure. For now we are just asking you to be responsible for the intuition.

### Exercises
*Save the answers to the following questions as 2.1.1 and 2.1.2 in answer.txt*
1. Name a source of static error that the low-rate I term can correct for.
2. Name two sources of dynamic error that the high-rate I term can. correct for.

## Problem 2: Preload the Throttle
The first step in the tuning process is finding an initial throttle value that allows your drone to have a smooth and controlled takeoff. To do this, you'll be adjusting the `throttle_low.init_i` in pid_class.py. This is the initial value of the low rate (slow changing) integral term for the throttle, which controls altitude. The default value is 60.

### Exercises
Test out different values for `throttle_low.init_i`, but be cautious because the drone could take off abruptly with a bad value. What happens when you set it to 20? What happens when you set it to 150?
* In one sentence, describe the drone's behavior as a result of changing the value up and down. *Save this answer as 2.2.1 in answers.txt*
* Now try to find the value for which your drone is able to have a smooth and controlled takeoff. This doesn't need to be perfect, and changing this value in increments of 10 and then 5 will get this value good enough. The goal is to reduce the overshoot and undershoot for the drone to takeoff and stay stable at 0.3m. *Save this value as 2.2.2 in answers.txt*


## Problem 3: Set the Trim
Next you will set the trim on roll and pitch. You will do this by tuning the low i terms to adjust for the static errors that exist on your drone. The default value is 0, and positive values will move the drone to the right or forward, and negative to the left or backward, depending on the axis you're modifying. Note that you may need to repeat this process periodically, for example after a crash or the like.
When performing this process, each time make sure that you:
* Place the battery in the same place each time as much as possible so the weight is distributed the same
* Tape or velcro the battery so it does not move
* Plug the flight controller while the struts are fully engaged and the drone is level, so the gyros are well calibrated.
* Calibrate the accelerometer
* Always place the drone so that the camera is closer to you and the skyline is farther away

### Exercises
1. Use a text editor (such as vim or nano) to modify pid_class.py to print out the low and high rate integral terms of the roll and pitch pids as follows:

    Find this block of code and uncomment the print statements as shown below
```python
# Print statements for the low and high i components
print "Roll  low, hi:", self.roll_low._i, self.roll._i
print "Pitch low, hi:", self.pitch_low._i, self.pitch._i
```
2. Perform one flight and record answers to the following questions. After the drone takes off, do not give it movement commands but allow it to drift. (Don’t allow it to hit anything though! You might have to kill or take over if it drifts a lot.) Note the following from this flight:
    * Which way does it drift?
    * How much does it drift? (Or in other words, roughly how long could you stay within a square meter of where you started?)
    * What are the low frequency I terms for roll and pitch after your flight?
    * Were they moving up or down?

At the end of the flight, the low rate I terms will have moved. Check what these values are by looking at \`4 in the screen. Set `self.roll_low._i` and `self.pitch_low._i` to these new values and repeat the process until you converge. Note that differences in the battery placement will cause enough change in the weight distribution to need very different initial values. You might need three or four flights until you converge. Typical values range between -20 and 20, but can also have a greater magnitude of depending on the weight distribution. *Save your roll and pitch low i terms as 2.3.1 in answers.txt*

## Problem 4: Implementing Your Altitude PID!
Now that the planar pids are tuned, and you have found a value for 'throttle_low.init_i' that allows the drone to take off at a reasonable rate, you will be using your altitude pid to control the height of the drone. To do this, we'll quit the pid_controller.py that runs in `4 of the screen, and we'll instead run student_pid_controller.py<sup id="a2">[2](#f2)</sup>. This will allow your PID to run alongside our planar pids, and on top of our throttle low i term which you found previously. Your pid will be responsible for keeping the drone flying steady vertically.

### Exercises
1. Prepare your drone to fly
2. Go to \`4 in the screen, press ctrl-c
3. Type 'python student_pid_controller.py' and press enter
4. Fly your drone!
5. Empirically tune your altitude pid gains on your drone as you did in the simulator portion of this project <sup id="a3">[3](#f3)</sup>

Take a video of your drone flying first using our altitude pid by running pid_controller.py in \`4, then take a video of your tuned pid by running student_pid_controller.py in '4. See if you can get yours to track the altitude setpoint better than ours! The drone should get to the setpoint quickly and stay there without bouncing up and down. *Submit these videos in Github Classroom as 2.4.1 and 2.4.2*

## Problem 5: Position Control
Once you have achieved good trim, you can try position control. Try to fly your drone for an entire battery without touching the controls! Do not try this until your I term preloads have been tuned as described above.

This video demonstrates the drone doing a zero velocity hover and drifting in the scene. Then we turn on position hold (you can tell because it drops the throttle as it takes over, and holds its position for several minutes.

Then we turn off the position hold so you can see it drift again, and then turn it on again at the end and land. You can tell when it is turned on because we move the drone back to the center of the flight area before each hold.

<iframe width="560" height="315" src="https://www.youtube.com/embed/WTohnsKs7dU" frameborder="0" allowfullscreen></iframe>

### Exercises

Engage position control in two steps. First you have to tell the drone to “remember” a frame. You can do this using the “r” key. This will save the frame at the drone’s current location. Next you have to engage or disengage position control. You can engage this mode with the “p” key, and disengage with 'v' for velocity control. So the procedure is to first save a frame (target location for the position hold) using “r” and then shortly after (before drifting too much) type “p”.

Position hold works best over a textured surface with lots of visual contrast. Even when doing position hold, always be ready to kill in case of a mishap. Especially be careful when looking at other windows.

### Try the following:
1. Engage position hold using the procedure described above. Observe the drone's behavior. How is it different from velocity mode?

2. How long are you able to hold position? Ideally you should be able to do this in one spot for an entire battery. If not, try retuning your I term preloads above.

3. Look in the vision tab in the screen. You should see it printing max_first_counter (and some other stuff). Pay attention to max_first_counter and observe whether/when it is seeing the first frame. Higher is better! Typical values are around 30 or 40, but a very good run might be 100 frames. Report your best max counts!

4. Turn off position hold with "v" and observe how the drone's behavior changes with position vs. velocity hold.

5. Try flying in velocity mode over a blank white posterboard. (There is one in 121.) Be careful! How does the drone's behavior change?

6. Now try over a uniform textured surface such as the floor in 121. Try a position hold. How well does it work? How long is it able to hold position?

Take a video of your drone flying in velocity control, and then engage position control. *Submit this video in Github Classroom as 2.5.1*



###### Footnotes
[<b id="f1">1</b>](#a1) It is important to do these tests over a textured surface so that the planar motion of the drone can be properly sensed using the optical flow vecotors of the camera (more on this in another section)

[<b id="f2">2</b>](#a2) You can make your future life easier by modifying pi.screenrc to always run your script by changing ```python pid_controller.py\n``` to ```python student_pid_controller.py\n```

[<b id="f3">3</b>](#a3) Try to just focus on the the altitude while doing this and ignore the planar motion because it is easiest to focus on one axis at a time when tuning the pids. The planar axes can be re-tuned after you tune your altitude pid if need be.
