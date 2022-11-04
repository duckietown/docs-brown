# Unscented Kalman Filter {#part:ukf-overview status=ready}

The fundamental issue of state estimation impacts widespread robotics
applications. Sensors give us an imperfect view of the world, as they
tend to contain noise in their readings. Similarly, when a robot moves
in the physical world, its motion has some amount of uncertainty,
deviating randomly from the ideal model that we predict it might
follow. Rather than ignoring these uncertainties and forming a naive
state estimate, we will be harnessing the power of probability to
yield mathematically optimal estimates using the Unscented Kalman
Filter (UKF) algorithm. The Kalman Filter (KF) algorithm and its
variants such as the UKF comprise part of the field of probabilistic
robotics, which aims to account for uncertainties that the robot will
inherently face as it interacts with the world with imperfect
information.  An entire course could be taught only on the topics of
filtering and state estimation.

In this project, we give a high-level overview of the necessary
foundations to understand the UKF algorithm. Then, you will implement
a UKF with a simple one-dimensional model of the drone's motion to
estimate its position and velocity along the vertical axis. Later on
in the project, we will expand the model to three spatial dimensions.
