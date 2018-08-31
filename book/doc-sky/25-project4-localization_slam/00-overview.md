# Project Four: Localization and SLAM {#part:localization_slam status=notready}
## Introduction

Having a good estimate of position is necessary for most tasks in autonomous mobile robotics. A self driving car, a delivery drone, or even a Roomba is not very useful without knowledge of its own location. In this project, we will be implementing two algorithms for localization on the PiDrone: **Monte Carlo localization** and **FastSLAM.**

These algorithms cover two important cases: one in which the robot has a map of its environment available beforehand, and a second in which it does not.  In this second case, the robot must use its sensors to simultaneously develop a map of its surroundings and localize itself relative to that map. Not surprisingly, this is referred to as the *simultaneous localization and mapping problem*, hereafter referred to as SLAM.
