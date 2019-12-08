# Project Checkoff  {#pid-checkoff status=ready}

## Functionality Check

1. The output of `step()` function in `student_pid_class.py` should be between 1100 and 1900.
2. The simulated drone should converge exactly at the setpoint and not oscillate for:    
- Idealized PID, `python sim.py`,  
- PID with latency, `python sim.py -l 6`, 
- PID with latency, noise and drag, `python sim.py -l 3 -n 5 -d 0.02`     

## Questions

You will be asked to answer one of the following questions:

1. In `step()` function in `student_pid_class.py`, which lines of your code relate to P/I/D term and how do you calculate u(t)?
2. In `reset()` function in `student_pid_class.py`, which variables you updated and why?
