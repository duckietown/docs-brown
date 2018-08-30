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
The error of the system $$e(t)$$, is calculated as the difference between the setpoint $r(t)$ and the process variable $$y(t)$$. That is:
