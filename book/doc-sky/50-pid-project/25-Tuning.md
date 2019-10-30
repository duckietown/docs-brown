# Part 2: Tuning {#pid-tuning status=ready}

Write brief answers to all exercises in <i>answers_pid.md</i>.

## Problem 1: Understanding the Controller
Our controller is a dual I-term (integral term) PID controller. The high-rate I-term changes quickly, allowing fast response to changes. The low-rate I-term changes slowly, allowing the drone to adjust to systemic sources of error. The provided PID gains have been pretuned to this drone hardware, and should not need significant modification for your specific drone. But, the initial low I-terms do need to be adjusted based on the static error of your specific drone.  

**Exercises**  
  1. Name a source of static error that the low-rate I term can correct for.  
  2. Name two sources of dynamic error that the high-rate I term can correct for.  
  
## Problem 2: Ziegler-Nichols Method
Imagine you are flying your drone and observing its flight for tuning it. Ideally, you would tune the $K_p$ by slowly increasing its value between flights until you can see the drone moving up and down with uniform oscillations. The final $K_p$ value that causes uniform oscillations is termed as $K_u$, the ultimate gain. While, the time difference between these two peaks during osciallations is termed as $T_u$, the ultimate period.
  
  **Exercises** 
   1. Given $K_u$ = 500, $T_u$ = 10. Use your $K_u$ and $T_u$ values to compute $K_p$, $K_i$, and $K_d$ using Ziegler-Nichols Method. 
   

## Problem 3: Flying with Velocity Control 
In velocity control, we use planar velocity measure from the camera as the process variable. The keyboard keys are used to set the setpoints.

  **Exercises** 
  1. Now suppose you are flying in velocity mode over a blank white poster board. How do you expect the drone to behave and why will it behave this way? Hint: think about why we fly over a highly textured planar surface.
