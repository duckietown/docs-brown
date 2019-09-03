# Localization and SLAM {#part:localization-slam status=ready}

## Introduction

Having a good estimate of position is necessary for most tasks in autonomous mobile robotics. A self driving car, a delivery drone, or even a Roomba is not very useful without knowledge of its own location. The task of determining the location of a robot is known as *localization.* In this project, we will implement two algorithms for localization on the PiDrone: **Monte Carlo localization** and **FastSLAM.**

These algorithms cover two important cases: one in which the robot has a map of its environment available beforehand, and a second in which it does not.  In this second case, the robot must use its sensors to simultaneously develop a map of its surroundings and localize itself relative to that map. Not surprisingly, this is referred to as the *simultaneous localization and mapping problem*, hereafter referred to as SLAM.

## Github Repository

Please use [this link](https://classroom.github.com/a/nS7a7Rg4) to generate your Github classroom repository and pull the stencil code. Use the Github repo created to handin your assignment and backup any changes you make.
