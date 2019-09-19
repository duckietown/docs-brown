# Assignment {#debugging-assignment status=ready}

Read Richard Feynman's 1974 Caltech Commencement address, entitled [Cargo Cult
Science]( http://calteches.library.caltech.edu/51/2/CargoCult.htm). In many
ways carrying out experiments in science is like debugging a robot. The example
of the experiment by Young in 1937 about rats running through mazes is
a beautiful example of debugging. In both cases you are carrying out
experiments in order to test hypotheses to determine the problem.

Below we present four strategies (and problems) for debugging that are useful
to try when you encounter a problem.

## Decompose the Problem

Decomposing the problem means breaking it down to smaller components.

For example, if your drone won't fly, try to decompose it into smaller problems.  
- Have you verified that each part works? Does the Pi power on?
- Is your flight controller talking to the motors?
- Can it connect to your laptop via CleanFlight?
- Does the IR sensor light turn on?

You want to try to isolate which parts are working and which parts are not
working in order to zero in on where the bug is. To decompose the problem it is
essential to be systematic and think through ways to check each part of
a system that is failing, separately.

For each of the below conditions:
1) Describe a test that verifies whether or not the component is functioning
2) Carry out your verification on the drone and describe the results of your
   test
3) Change something about the drone so that just that component stops working
   and describe what you changed to cause it to stop working
4) Carry out your test again and describe the results

*Write your answers in `answers.md` for each condition.*

1. The motors are powering on.

2. The Pi is receiving data from the camera

## Visualize the State

To figure out what is wrong, it helps to visualize the state of the robot and
the system.
- Can you see the output from each sensor?
- Is the output what you expect?
- Is there a "human friendly" way to draw what is going on?

Often one spends as much time writing visualizers as one does implementing the
algorithm or functionality on the robot. An incorrect transform might be
impossible to debug if you print out the matrix, but instantly obvious as soon
as you draw it in a 3D visualizer.

For each condition below:
1) Use the Javascript interface to visualize the state or output
2) Describe a procedure to verify the output works as expected
3) Carry out the procedure using your drone and describe the results

*Submit screenshots from the Javascript interface as visualize_[INSERT CONDITION NUMBER].png and write the written
responses in `answers.md`.*

1. Camera Output: How accurate are the position estimates over different surfaces? 

2. IR Sensor Readings: How does it work with different materials? What are
   the minimum and maximum distances?

## Break the Abstraction Barriers

Bugs don't respect abstraction barriers, and you shouldn't either! [The Law of
Leaky Abstractions](https://www.joelonsoftware.com/2002/11/11/the-law-of-leaky-abstractions/) applies here. As you decompose the problem, you might find
that all the code you wrote is correct, and the actual bug lies in some other
module. In the course of developing the drone, we had to fix bugs caused by
insufficient swap space on the Raspberry Pi, incorrect implementation of the
MSP serial protocol used to talk to the drone, and more. If decomposition tells
you that all your parts are working, then continue working to isolate and find
the problem in some other module, even if you didn't write it.

In embedded computing, often the LEDs give important information about the
underlying components, as do audible beep codes. Note that the CleanFlight
software and the ESCs spin the motor at high frequencies in order to generate
audible beeps. *Write the answers to the following questions in `answers.md`.*

1. Find the LEDs on the Rasberry Pi. What does each LED mean? What do they
   mean? What happens to the LEDs if the SD card is not plugged into the Pi?

2. Find the manual for the Skyline 32. What LEDs does it have? What do they
   mean? What happens if the Skyline is not receiving power?

3. Find the manual for the ESCs. (We couldn't find the 12A manual so use the
   one for 30A.) What mechanisms do the ESCs have to indicate their status?

## Slow Things Down

Things happen fast on a robot, often too fast to see. It helps to find ways to
slow things down. You can look at a recorded log of what happened and play it
back slowly. Or you can write software to carry out each step in isolation and
slowly enough that you can verify its correctness.

In order to fly, the drone must read sensor data and output motor commands at
a very high frame rate. However it is often hard to see what is happening since
it is changing so fast. For both of the following assignments, you should not
need to write any ROS code. In both cases we are looking for relatively short
programs that talk to the respective hardware module.

1. Write a program to arm the drone, wait 10 seconds, and disarm the drone.
   Verify your program runs. Look at `h2rMultiWii_test.py` for an example of
   how to talk directly to the controller without using ROS. The flight
   controller speaks [Multiwii Serial
   Protocol]("http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol).
   *Submit your answer as `my_arm.py`*.

2. Write a program to read a single frame from the camera, save it to a file,
   and return, without using ROS. Verify your program runs, and include your
   picture in the project write-up. *Submit your answer as `my_frame.py`.*


## Handin

Create your Github repo using [this link](https://classroom.github.com/a/YmNdQdha).

Handin the following files:

- answers.md
- my_arm.py
- my_frame.py
- visualize_1.png
- visualize_2.png
- visualize_3.png
