# Appendix C: Position Control {#pid-position status=ready}

Thus far, the planar PIDs have been used to control the velocity of the drone; now, you will use cascaded PIDs to control the position of the drone. The cascaded PIDs are set up so that the position controller forms the outer loop which uses the position error to provide setpoint velocities for the inner loop velocity controller.

### How to Engage Position Control
Engaging position control involves two steps. First you have to tell the drone to “remember” a frame. You can do this using the _r_ key. This will save the frame which the drone will attempt to fly directly above. Next you have to engage position control. You can engage this mode with the _p_ key, and disengage with _v_ for velocity control. So the procedure is to first save a frame (target location for position hold) using _r_ and then shortly after (before drifting too much) type _p_.

**Note:** Position hold works best over a textured surface with lots of visual contrast. Even when doing position hold, always be ready to kill in case of a mishap. Especially be careful when looking at other windows.

### Position Control Demo
This [video](https://www.youtube.com/embed/WTohnsKs7dU) demonstrates the drone doing a zero velocity hover and drifting in the scene. Then we turn on position hold (you can tell when it is engaged when the drone's throttle drops) and it holds its position for several minutes.

Then we turn off the position hold so you can see it drift again, and then turn it on again at the end and land. You can tell when it is turned on because we move the drone back to the center of the flight area before each hold.

## Flying with Velocity Control
First, you are going to experiment with flying your drone in velocity control and controlling its motion with the keyboard keys. Based on observations and knowledge of the controllers, you will then explain the inner workings of the velocity PIDs in your own words.  

**Setup**  
Prepare your drone to fly over a highly textured planar surface. Make sure there is space for the drone to fly around.  
**Exercise**  
Fly your drone in velocity control (the default control) and make sure there is room to fly to the right. Press and hold 'L' and observe the drone's motion, and release 'L' to stop the drone from moving.  

1. Explain what the following key terms are in this controller, and how they change to cause the drone to move when you press 'L' and stop when you release: setpoint, error, control variable, process variable, proportional term, integral term, derivative term. We are looking only for a higher level description to demonstrate understanding of the PID controllers.  
2. Try flying in velocity mode over a blank white poster board. Be careful! What do you notice about the drone's behavior, and what do you suspect causes this?  


## Flying with Position Control
Now you are going to fly your drone in position control and experiment with controlling its motion with the keyboard keys. Based on observations and knowledge of the controllers, you will then explain the inner workings of the position PIDs in your own words.  
**Setup**  
Prepare your drone to fly over a highly textured planar surface. Make sure there is space for the drone to fly around.  
**Exercises**  
1. Engage position hold using the procedure described above. Observe the drone's behavior. How is it different from just velocity control?  
2. How long are you able to hold position? Ideally you should be able to do this in one spot for an entire battery. If not, try re-tuning your I-term preloads above. If you're flying on the power supply instead of a battery, the drone should stay in place indefinitely, but you can stop it after 5 minutes.  
3. While flying in position control, make sure there is room for the drone to fly to the right and then take note of the desired position in \`4 of the screen. Now press the 'L' key in the user interface and note the new desired x-position of the drone; it should be 0.1m to the right of the drone's last position. Explain what the following key terms are in the outer loop position controller, and how they change to cause the drone to move and stop 0.1m to the right after you press 'L' in position control: setpoint, error, control variable, process variable, proportional term, integral term, derivative term. We are looking only for a higher level description to demonstrate understanding of cascaded PID controllers.  
4. Try flying in position control over a uniform surface such as the floor in 121, or un-patterned carpet. Echo the state of the drone by typing `rostopic echo /pidrone/state` into an empty window in the screen. Note the position data, and explain your observations of how well the drone is able to estimate its position. How long is it able to hold position? Does the drone move correctly when you use the arrow keys?
