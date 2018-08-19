# Assignment {#assignment status=ready}

This assignment gives an introduction to our course and reviews some
basic material you will need.

Please write your answers to be submitted in a PDF format. We reccomend [LaTeX](https://www.sharelatex.com/) or MarkDown, however scanning _legible_ handwriten answers is also acceptable. 

When you are done, please email your answers to cs1951rheadtas@lists.brown.edu. The subject of the email shoud be **[cs login] Assignment 1**. The email should include the attachments

* [cslogin]_hw1.pdf 
* collab_policy.pdf
* safety_policy.pdf


## Collaboration Policy

Please read and sign the [collaboration policy for CS1951R](collab_policy.pdf). You must turn in the signed pdf with the assignment if you wish to be graded.

## Safety Policy
Please read and sign the [safety policy for CS1951R](safety_policy.pdf). You must turn in the signed pdf with the assignment if you wish to be graded.

## Problem 1 (20 points)
Before you start putting a lot of time into this course, it is
important to figure out what you will get out of the course.  Think
about what you expect to learn from this course and why it is worth
investing a lot of time.  What do you hope to learn that you can take
away for the next ten or twenty years of your career?   

I. What is a robot? What is a machine? What is a vehicle? \\
II. Is a car a robot? How is my drone a robot?\\
III. If I can fly a drone by remote, what can I get out of programming it?\\


## Problem 2 (20 points)

For this problem we strongly recommend you do these calculations by
hand, because they are warmup questions designed to remind you of some
of the prerequisite material for the class.
<ol>
<li> Multiply the matrix by the following vector:
$$\begin{bmatrix}1 & 0 & 1\\0 & 1 & 2\\0 & 0 & 1\end{bmatrix} \times \begin{bmatrix}0\\0\\1\end{bmatrix}$$
<br/><br/>

### Answers {.hidden}
$$\begin{bmatrix}1 \\ 2 \\ 1\end{bmatrix}$$
<br/><br/>

</li>
<li> Multiply the matrix by the following vector:
$$\begin{bmatrix}0\\0\\1\end{bmatrix} \times \begin{bmatrix}1 & 0 & 4\\0 & 1 & 10\\0 & 0 & 1\end{bmatrix} $$
<br/><br/>

### Answers {.hidden}
$$\begin{bmatrix}4 \\ 10 \\ 1\end{bmatrix}$$
Also acceptable: undefined, since if the vector is treated as a matrix, there's no way to do this calculation. However, the problem stated that it was a vector.
<br/><br/>

</li>
<li>Imagine a robot is at $$(0,0)$$.  It uses a sensor to detect an
object on a distance of $$10\mbox{m}$$ and a heading of
$$45^{\circ}$$.  Where is the obstacle relative to the robot?  Give
coordinates and draw your answer on a map.
<br/><br/>

### Answers {.hidden}
The obstacle is at $$(5\sqrt{2},5\sqrt{2})$$.  
<br/><br/>

</li>
</ol>


## Problem 3 (20 points)

Read the <a href="https://www.faa.gov/uas/">FAA website</a> on
Unmanned Aircraft Systems.  Provide short answers to the following
questions.

### Answers {.hidden}

---

<b>Here is a little bit of discussion of the relevant rules:</b>
Tl;dr - We are in Class G airspace, that classification doesn't directly matter, and we need to notify Rhode Island Hospital before flying.

We had some questions come up about airspace classification on homework 1. In an attempt to make sure we are all on the same page, I've done some more digging on the subject. This is the relevant FAA webpage: https://www.faa.gov/uas/faqs/. Because we fall under the Special Rule for Model Aircraft, the actual classification of the space we fly does not matter at all. That is only relevant for commercial or licensed flying under section 107. The airspace rules that apply to us are:

1. We must notify airports or heliports within 5 miles (i.e. Rhode Island Hospital).
2. We must fly within line of sight.

Class C airspace applies within 5 nautical miles of airports up to 4,000 feet. It also applies within 10 nautical miles between 1,200 feet and 4,000 feet. [Source](https://en.wikipedia.org/wiki/Airspace_class_(United_States)#Class_C)

This means that the CIT is not in Class C for altitudes below 1,200 feet. If we fly high enough, we would be in Class C because we are within 10 miles of the airport. However, presumably at that height we would no longer be in line of sight (assuming by line of sight they mean within visible range). Therefore, we are indirectly restricted from Class C airspace, even though the Classes don't directly apply to us.

There's a final interesting FAQ on the FAA site:

_Can an airport operator object to model aircraft flights near an airport?_
"Yes, an airport operator can object to the proposed use of a model aircraft within five miles of an airport if the proposed activity would endanger the safety of the airspace. However, the airport operator cannot prohibit or prevent the model aircraft operator from operating within five miles of the airport. Unsafe flying in spite of the objection of an airport operator may be evidence that the operator was endangering the safety of the National Airspace System. Additionally, the UAS operator must comply with any applicable airspace requirements."

This seems to imply that you can fly regardless of what the air traffic control says, but that you could then be arrested for it. This seems like a really weird addendum, like maybe they are trying to regulate things that Congress hasn't actually given them jurisdiction over.

Bottom line, as long as we notify Rhode Island Hospital it seems like we are ok.

---



<ol>

<li>  What class of airspace is the area around the CIT?    (You might find it easiest to use the <a href="https://www.faa.gov/uas/where_to_fly/b4ufly/">B4UFLY Smartphone App</a>.)
<br/>
### Answers {.hidden}
Class G
</li>
<li>  What is the closest airport to the CIT?  
<br/>

### Answers {.hidden}
<a href="http://www.pvdairport.com/">T.F. Green Airport</a> and the <a href="https://www.airnav.com/airport/RI25">Heliport at the Rhode Island Hospital</a>
</li>
<li>  Is it okay to fly your drone outside the CIT?   Why or why not? 
<br/>

### Answers {.hidden}
It is okay for students to fly outside as long as they respect the FAA airspace rules.  This falls as recreational use.   However the CIT is within five miles of the Heliport at the Rhode Island Hospital.  As a result it is not okay to fly without coordianting with them. 

</li>
<li>  What are some risks of drone flight?  How could people get hurt with the robot?
<br/>

### Answers {.hidden}
People can get hurt when it falls on them.  A propeller hits them.  A propeller breaks off and gets in their eye.  The battery explodes.  

</li>
</ol>

## Problem 4 (20 points)

Pick a robot that was used to solve a real-world problem.  You might
choose the Baxter robot, the <a
href="https://en.wikipedia.org/wiki/Curiosity_(rover)">NASA Curiosity
Rover</a>, the <a href="https://waymo.com/">Waymo robot</a>, the <a
href="https://en.wikipedia.org/wiki/Roomba">iRobot Roomba</a> or another robot of your choice.  Answer the following questions about the robot you pick: 

<ol>
<li> What problem is the robot solving?  Your answer should not
reference the robot at all but talk about the problem that people had,
why it was hard, and why it is important to solve it. 

### Answers {.hidden}
There is no one right answer to these questions, but we
will provide our answers for the course drone.  We had a need for a
robot that would grab student interest, be applicable to real-world
problems, be easy to program, and teach students the fundamentals of
robotics.  It is hard to find a platform that satisfies all of these
needs - we found other drones that were not open enough to allow us to
teach the skills we wanted.  We considered non-drone platforms but
felt that a flying platform was more engaging and more real.
<br/><br/>


</li>

<li>  How does the robot solve the problem?  What changes does the robot
make to the physical world to solve the problem?
### Answers {.hidden}
There is no one right answer to these questions, but we
will provide our answers for the course drone.  We had a need for a
robot that would grab student interest, be applicable to real-world
problems, be easy to program, and teach students the fundamentals of
robotics.  It is hard to find a platform that satisfies all of these
needs - we found other drones that were not open enough to allow us to
teach the skills we wanted.  We considered non-drone platforms but
felt that a flying platform was more engaging and more real.
<br/><br/>

</li>

<li>What sensors does the robot use to solve the problem?  What does
it need to know about its environment, and how does it find out?

### Answers {.hidden}
The drone is equipped with an IMU and gyro which allows
it to set angular accelleration and translational accelleration.
Additionally it has an IR sensor pointed downward in order to sense
its height.  It also has a camera pointed downward in order to sense
its position and velocity.  It needs to know its position, velocity,
and accelleration for flight in indoor obstacle-free environments.
<br/><br/>

</li>

<li>What actuators does the robot use to solve the problem?  What does
it need to change about its environment, and how does it change it?</li>

<li>How well does the robot work to solve the problem?  How is its
performance evaluated?</li>

<li>How does the robot fail?  What happens when it fails? </li>

<li>How much does the robot cost?  For some robots these numbers may
not be directly available; do some digging and try to find out.  If
you cannot find out, try to estimate the cost and explain the
reasoning behind your estimate.

</li>
</ol>



## Problem 5 (20 points)

Read the <a
href="https://www.joelonsoftware.com/2002/11/11/the-law-of-leaky-abstractions/">The
Law of Leaky Abstractions</a>.  How might this be especially relevant
to robotics?

Think about some ways that implemented systems might not be true to
their modeled behavior. How can we plan with abstractions despite
these challenges?

### Answers {.hidden}

Robotics is especially suceeptible to the Law of Leaky Abstractions
because for a robot to work, all aspects of the robot must work as
well: networking, systems, memory management, processor heat.  We have
had PIs fail due to using too much CPU, causing the processor to
overheat, as well as bugs in code other people wrote, such as reading
the IR sensor.  


## Handin

When you are done, please email your answers to cs1951rheadtas@lists.brown.edu. The subject of the email should be **[cs login] Assignment 1**. The email should include the attachments

* [cslogin]_hw1.pdf
* collab_policy.pdf
* safety_policy.pdf

