# Assignment {#rcflying-assignment status=ready}

In this assignment, you'll briefly research control algorithms that are used on quadrotors, and then you will fly an airplane and a quadrotor in simulation to get a feel for what these control algorithms are doing.

## Control Algorithms
For the first part of this assigment, read up to 3.2 of [this article](https://arxiv.org/pdf/1602.02622.pdf), and then pick two other control algorithms to compare and contrast with a PID controller (what we use on our drones). Identify the major advantages and disadvantages of each algorithm. Do not worry about understanding all of the math derived in the article; we have asked you to read it once to be exposed to it, but such a model it is not required for the PID controller. If one wished to implement a more advanced algorithm, then a model such as the one described in the article would need to be created.


## Fly with RC!
In this part, you will fly an airplane and a quadcopter in simulation
using an RC controller. Doing so will give you an intuitive sense for roll, pitch,
and yaw, as well as how they affect the dynamics of aerial vehicles.

You will be using the RealFlight 7.5 simulator along with the associated RC
controller, which are the same controls used to fly a real physical
aircraft over RC.

Go to the 8th Floor SciLi RealFlight workstation and fly using the RC controller
and the flight simulator! The workstation is located at the monitor next to the Baxter.
We only have one workstation for the class, so you can reserve a time [here](https://calendar.google.com/calendar/selfsched?sstoken=UUlhN0JXdzRFM0RHfGRlZmF1bHR8NGNlMjk2ODA1NzJhZmRiOTNmMjcwNzUzMjQ4MTdiZWE) to use it.

The simulator is installed on the computer at the workstation. If it is not open
already, open the RealFlight7.5 Launcher (on the Desktop) and click Run RealFlight.

The controls you will be using are:
<figure>
    <figcaption>RC Controls</figcaption>
    <img style='width:35em' src="rc.png"/>
</figure>

Create a file called `answers.md` in your RC Flying Github repo (see section
1.7 Handin for the link to generate your repo). Write all answers to the
following questions in this file.

## Airplane

Flying an RC airplane is nice because it gives an intuitive sense for
the controls and roll, pitch, and yaw since the airplane's body is
asymmetric.  Choose one of the planes and fly in the sim using the
controls to get a feel for the system.

1. Can you hover in one place with the airplane?  Why or why not?

## Quadcopter X (classic) in Acro Mode

Go to the airport "Joe's Garage HD" in the "Sierra Nevada" section.
From the bar at the top choose _Aircraft_ then _Select Aircraft_ and choose
the _Quadcopter X (Classic)_. On the controller, flip switch number 7
(they're labeled) to the **B** position.  In this mode, the aircraft
has a gyroscope to hold its angle, but it does not have an
accelerometer for automatic leveling. You will be controlling the
throttle and the angular velocities directly.

1. Use the throttle to take off.  Describe in words what this does to
each of the four motors.  What effect does this action have on the
drone?

2. Now take off. Fly around a bit and try to land back on the target.
Try to do a loop-de-loop.  Don't worry if you crash a lot; that's part
of the point of a simulator; you can take off again just by hitting
the red "reset" button.  Report on your experience.  Is this easy or
hard? Why?

## Quadcopter X in Stabilized Mode

Now select Quadcopter X from the Aircraft menu. Flip switch number 8
to position **B**.  In this mode the aircraft uses its accelerometer
for automatic leveling, just like our Skyline.  In fact, you can plug
an RC antennae into the skyline and control it with an RC controller
in just this way.

1. Fly around in a circle and land back on the target. Is this easier or harder
than the previous mode? Why?

2. Why might this mode require an accelerometer, if the previous mode only required a gyro?

## Quadcopter X in Pos-Hold Mode

Flip switch 8 to position **A** to enable pos-hold. Fly around to get
a sense of the aircraft dynamics. Try flying in a direction quickly
and then stopping; observe the differing behavior between modes **A**
and **B** on switch 8.

1. Fly around in a circle and land back on the target. Is this easier or harder
than the previous mode? Why?

2. Try to fly in a loop-de-loop. Can you do it?  Why or why not?

3. If you were to write a controller algorithm that passed commands to a quadcopter in stabilized mode (<b>B</b>) to make it behave like a quadcopter in pos-hold mode (<b>A</b>), what information would you need? What sensors could you use to obtain that information?

## Quadcopter Trials Challenge

To get familiar with flying, first just practice getting the quad to hover using only throttle (up/down on the left stick). Now experiment with roll and pitch (up/down left/right on the right stick). Finally, try using yaw (left/right on the left stick). Note that the roll and pitch commands are relative to the orientation of the drone. If you crash and need to reset the simulator, press the spacebar, or you can push the reset button on the controller.

1. Go to Challenges and try out the quadcopter trials.  This
challenge uses a similar auto leveling quadcopter.  How far can you
get?  Stefanie got stuck at Level 4.

## Have fun!

Feel free to play with the simulator as long as you like and try out
some of the other aircraft.

1. Write a brief report about what you tried, and let us know the coolest
activity or feature that you found.

## Handin
Use [this link](https://classroom.github.com/a/ZZYbHx_8) to create your RC Flying Github Repo. After finishing the
assignment, commit and push `answers.md` to this repo before the deadline.
