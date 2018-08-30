# PID Controllers

## Introduction
A PID (proportional, integral, derivative) controller is a control algorithm extensively used in industrial control systems to generate a control signal based on error. The error is calculated by the difference between a desired setpoint value, and a measured process variable. The goal of the controller is to minimize this error by applying a correction to the system through adjustment of a control variable. The value of the control variable is determined by three control terms: a proportional term, integral term, and derivative term.

## Characteristics of the Controller

### Key Terms and Definitions
* **Process Variable**: The parameter of the system that is being monitored and controlled.  
* **Setpoint**: The desired value of the process variable.  
* **Control Variable/Manipulated Variable**: The output of the controller that serves as input to the system in order to minimize error between the setpoint and the process variable.  
* **Steady-State Value**: The final value of the process variable as time goes to infinity.  
* **Steady-State Error**: The difference between the setpoint and the steady-state value.  
* **Rise Time**: The time required for the process variable to rise from 10 percent to 90 percent of the steady-state value.  
* **Settling Time**: The time required for the process variable to settle within a certain percentage of the steady-state value.  
* **Overshoot**: The amount the process variable exceeds the setpoint (expressed as a percentage).  

### General Algorithm
The error of the system $$e(t)$$, is calculated as the difference between the setpoint $$r(t)$$ and the process variable $$y(t)$$. That is:

> $$e(t) = r(t) - y(t)$$

The controller aims to minimize the rise time and settling time of the system, while eliminating steady-state error and maximizing stability (no unbounded oscillations in the process variable). It does so by changing the control variable $$u(t)$$ based on three control terms.

#### Proportional Term
The first control term is the proportional term, which produces an output that is proportional to the calculated error:

> $$P = K_pe(t)$$

The magnitude of the proportional response is dependent upon $$K_p$$, which is the proportional gain constant. A higher proportional gain constant indicates a greater change in the controller's output in response to the system's error.

#### Integral Term
The second control term is the integral term, which accounts for the accumulated error of the system over time. The output produced is comprised of the sum of the instantaneous error over time multiplied by the integral gain constant $$K_i$$:

> $$I = K_i\int_0^t\!e(\tau)\,\mathrm{d}\tau$$

#### Derivative Term
The final control term is the derivative term, which is determined by the rate of change of the system's error over time multiplied by the derivative gain constant $$K_d$$:

> $$D = K_d\frac{de(t)}{dt}$$

#### Overall Control Function
The overall control function can be expressed as the sum of the proportional, integral, and derivative terms:

> $$u(t) = K_pe(t) + K_i\int_0^t\!e(\tau)\,\mathrm{d}\tau + K_d\frac{de(t)}{dt}$$

In practice, the discretized form of the control function may be more suitable for implementation:

> $$u(t) = K_pe(t_k) + K_i\sum_{i=0}^k e(t_i)\Delta t + K_d\frac{e(t_k)-e(t_{k-1})}{\Delta t}$$

The figure below summarizes the inclusion of a PID controller within a basic control loop. 

![]
