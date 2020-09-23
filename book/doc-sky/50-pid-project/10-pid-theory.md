# PID Theory Questions    {#pid-theory status=ready}

<minitoc/>

## The True Value and Error Curves
The figure below shows a true value curve for a PID controller. Draw the corresponding error curve for this graph. You can draw by hand and upload the picture. (Hint: refer to the error definition equation from before)


<figure>
    <figcaption>True Value Curve for A PID Controller. The orange dot line indicates the setpoint and the black line is the true value curve.</figcaption>
    <img alt='true value curve' style='width:35em' src="true_value_curve.png"/>
</figure>


## Explain an Effect

<ol type="1">
  <li>Answer the following questions (3-5 sentences each):
  <ol type="a">
    <li>What will happen when the absolute value of $K_{p}$ is very large? What will happen when the absolute value of $K_{p}$ is very small?</li>
    <li>Can $K_{p}$ be tuned such that the $P$ term stops oscillations? Why or why not?</li>
    <li>Can the process variable stabilize at the setpoint (i.e. zero steady-state error) with only the $P$ term and the $D$ term? Why or why not?</li>                                      
  </ol></li>       
  <li>Explain the following effects caused by $K_{p}$, $K_{i}$ and $K_{d}$ (3-5 sentences each). For example, here is a sample answer (though you do not need to follow the pattern):
  <ul>
    <li>[Q:] *The rise time decreases when $K_{d}$ increases.*</li>
    <li>[A:] *When $K_{d}$ increases, the error at time step $t+1$ decreases. This is because larger and larger $K_{d}$ results in larger and larger control signals at time step $t$. This drives the system to achieve a lower error at time step $t+1$. As the error at time step $t+1$ decreases, the slope of the true value curve increases. Since the slope increases, the rising time towards the setpoint should decrease (slightly).*</li>
  </ul>
  <ol type="a">
    <li>The rise time decreases when $K_{p}$ increases.</li>
    <li>The settling time increases when $K_{i}$ increases.</li>
    <li>The overshoot decreases when $K_{d}$ increases.</li>                                   
  </ol></li>                                   
</ol>


## Start Tuning

When designing a PID controller, it is important to choose a good set of $K_{p}$, $K_{i}$, and $K_{d}$; poor choices can result in undesirable behavior. The graphs in the figure below illustrate behavior resulting from unknown sets of $K_{p}$, $K_{i}$, and $K_{d}$. In each graph, the orange dot line indicates the setpoint and the black line is the true value curve. For each graph, answer the following (1-2 sentences each):


1. Which term(s) went wrong, if any? In other words, which term(s) are too high or too low?
2. How can you correct the behavior?



<img src="tuning1.png" alt='tuning curve 1' style='width:35em' />
<img src="tuning2.png" alt='tuning curve 2' style='width:35em' />
<img src="tuning3.png" alt='tuning curve 3' style='width:35em' />
<img src="tuning4.png" alt='tuning curve 4' style='width:35em' />



## PID on the PiDrone
Sometimes a PID controller will have an extra offset/bias term $K$ in the control function (see the equation below). For the drone, this $K$ is the base throttle needed to get the drone off the ground.

$$u\left ( t_k \right ) = K_{p}e\left ( t_{k} \right )+K_{i}\sum_{i=0}^{k}e\left ( t_{i} \right )\Delta t+K_{d}\frac{e\left ( t_{k} \right )-e\left ( t_{k-1} \right )}{\Delta t}+K$$

### Altitude Control
Suppose you are implementing an altitude PID controller for your drone (i.e. up/down movement).


1. If the *setpoint* is the desired height of the drone, then what is the *process variable*, the *error* and the *control variable* for the altitude PID controller?
2. What could happen if $K$ is set too high?


**Note:** We are looking only for a higher level description to demonstrate understanding of the PID controllers.

### Velocity Control
Suppose you are implementing a velocity PID controller for your drone. In this case, the drone only moves forward/backward and left/right. For example, fly your drone in velocity control mode (the default) and observe the following behavior: 1) press and hold ‘L’ for a few seconds, then 2) release ‘L’ to stop the drone from moving.


1. What is the *setpoint*, *process variable*, *error* and *control variable* for the veloity PID controller?
2. How do these key terms change to cause the drone to move when you press ‘L’?

**Note:** We are looking only for a higher level description to demonstrate understanding of the PID controllers.
