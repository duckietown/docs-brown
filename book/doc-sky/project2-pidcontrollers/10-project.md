# Project 2: Implementing an Altitude PID Controller

In this project, you will be implementing a PID controller
in `student_pid_class.py` and using a simulator,
`sim.py` to test its behavior.

However for this project you should implement the discrete version:

$$K_P, K_I, K_D, K = \mbox{Constants and Offset Term}$$

$$e_t = \mbox{error at time } t$$

$$\Delta t = \mbox{time elapsed from previous iteration}$$

$$ u_t = K_P(e_t) + K_I \sum_{j=1}^t(e_t \Delta t_t) + K_D (\frac{e_t - e_{t-1}}{\Delta t_t}) + K$$


You should run the simulator on your base station or on a department computer and not on your drone, since it requires a graphical user interface to visualize the output. You can run the git clone command wherever you plan to work. You will need numpy and matplotlib: `pip install numpy matplotlib pyyaml`. To run the simulator, use `python sim.py` and the PID class in _student_pid_class.py_ will automatically be used to control the simulated drone. The _up_ and _down_ arrow keys will change the set point, and _r_ resets the sim.


## Problem 1:  Implement Idealized PID
Write brief (one sentence is fine) answers to the following questions in your _altitude_answers.pdf_. You will implement _student_pid_class.py_ and tune the parameters in z_pid.yaml:

<ol>

<li>Implement the step function to return a constant $K$. At what value of <i>K</i> does the drone takeoff? Can you get it to hover? Set $K$ to 1300 for questions 2 and 3.
{% if include.show_answers %}
<br/>
<b>Answer:</b>
Takeoff value??
Yes if you get exactly the right value in this idealized problem, but even very small differences off will either not take off, or will ascend indenfinately because there is no feedback.
<br/>
<br/>
{% endif %}
</li>

<li>Implement the P term. What happens when the absolute value of the P term is very large?
What happens when its absolute value is very small?  Can you tune the P term to stop
oscillations?  Why or why not?  
{% if include.show_answers %}
<br/>
<b>Answer:</b>

When it is large, you get big fast oscillations.  When it is small you
get lower amplitude oscillations.  You cannot stop oscillations with P
alone in this idealized problem because there is no latency.  In some
domains though, there is an implicit D term from momentum and air
resistance, so you can stop oscillations or have very small ones with
P alone.

<br/>
<br/>

{% endif %}
</li>

<li>Implement the D term.  Set the $K_I$ and $K_P$ terms to zero.  What happens?
{% if include.show_answers %}
<br/>
<b>Answer:</b>

With D alone it does not send commands when the error is not changing
so the drone doesn't move.

<br/>
<br/>
{% endif %}
</li>


<li>Now implement the derivative term. Set $K$ to 1300 and tune the
$K_P$ and $K_D$ so that the drone comes a steady hover. Describe the
trade off as you change the ratio of $K_P$ to $K_D$. Does the drone
stabilize at its target? Why or why not?

{% if include.show_answers %}
<br/>
<b>Answer:</b>

If $K_P$ is much larger than $K_D$, the drone will oscillate for a
long time.  If $K_D$ is very large, the drone will take longer to
reach steady state.

<br/>
<br/>

{% endif %}
</li>


<li>Implement the integral term and observe the difference between PD and PID control. What role does the i term play in this system? Set the D and P terms to zero.  What happens?
{% if include.show_answers %}
<br/>
<b>Answer:</b>
The i term accounts for the steady state error in the system, and allows it to overcome constant forces like gravity.
The I term slowly drives the drone to the set point but does not move quickly without P and D.
<br/>
<br/>

{% endif %}
</li>

<li>Finally, try setting $K$ higher and lower. What is the relationship between $K$ and $K_I$ (especially when the drone first takes off)? What could happen if $K$were set too high on the real drone?
{% if include.show_answers %}
<br/>
<b>Answer:</b>
$K$ controls the bias term that is always added to the controller.   It will cause the drone to take off faster, and make all the other terms smaller.  However if it is too high, it could cause the drone to hit the ceiling.  Eventually the $I$ term would correct for $K$ being too large, but not necessarilly before the drone crashes.
<br/>
<br/>

{% endif %}


</li>
</ol>

Tune the constants in your PID controller to the best of your
abilities. The drone should chase the set point very closely, and
should not wobble when the set point is still.

**NOTE**: Pay careful attention to the behavior of _reset_. This has
  been the source of a lot of crashes in our lab; see how many
  problems you can anticipate reset causing and how many you can
  prevent.

## Problem 2:  Implement PID with Latency

Now, we introduce latency!  Run the sim as `python sim.py -l 6` to
introduce 24 milliseconds of latency (six steps of latency running at
25 hz).  Tune the constants in your PID controller to the best of your
abilities. The drone should chase the set point very closely but
converge more slowly and you should get it to stop oscillations.
Report your PID terms before and after. Which terms changed the most?
Why would latency affect that term/those terms in particular?
{% if include.show_answers %}
<br/>
<b>Answer:</b>
Latency will cause the $D$ term to need to be tuned smaller, because
the derivative will lag and cause the drone to over correct.
<br/>
<br/>

{% endif %}

Run `python sim.py -h` to see the other simulator parameters. We
encourage you to experiment with those as well and observe their
effects on your controller.


## Problem 3:  Implement PID with Latency, Noise, and Drag

In the most realistic mode, you will tune a controller with latency,
noise, and a drag coefficient.  You can do this with the command line
arguments `python sim.py -l 3 -n 0.5 -d 0.02` to be most realistic to
real world flight. Tune with these arguments to be as good as
possible.
{% if include.show_answers %}
<br/>
<b>Answer:</b>
We should include our values.
<br/>
<br/>

{% endif %}


## Checkoff:

Come see a TA to show off your PID controller from problem 3!  We will
assess the speed it converges (dynamic performance after a change to
the set point) and the steady-state performance.  It should converge
without oscillations in the steady state, and it should quickly (a
few seconds) converge to the new set point.
