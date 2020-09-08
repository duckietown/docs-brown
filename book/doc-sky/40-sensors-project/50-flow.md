# Estimating Velocity by Summing Optical Flow Vectors {#sensors-assignment-optical-flow status=draft}

We want to estimate our $x$ and $y$ velocity using the PiDrone's camera. Thankfully, optical flow from the PiCamera is calculated on board the Raspberry Pi. All we have to do is process the optical flow vectors that have already been calculated for us!

To calculate the $x$ velocity, we have sum the $x$ components of all of the optical flow vectors and multiply the sum by some normalization constant. We calculate the $y$ velocity in the same way. Let $c$ be the normalization constant that allows us to convert the sum of components of optical flow vectors into a velocity.

How do we calculate $c$? Well, it must have something to do with the current height of the drone. Things that are far away move more slowly across your field of view. If a drone is at a height of .6 and a feature passes through its camera's field of view in 1 second, then that drone is moving faster than another drone at a height of .1 whose camera also passes over the same feature in 1 second. If we let $a$ be the altitude of the drone, then the drone's normalization constant must be $c = ab$, where $b$ is some number that accounts for the conversion of optical flow vectors multiplied by an altitude to a velocity. You do not have to worry about calculating $b$ (the \emph{flow coefficient}), as it is taken care of for you.

In summary, to calculate the $x$ velocity, we have to sum the $x$ components of the optical flow vectors and then multiply the sum by $ab$. The $y$ velocity is calculated in the exact same way.

## Questions
1.  The Pi calculates that the optical flow vectors are [5 4], [1, 2], and [3, 2]. The flow vectors are in the form [$x$-component, $y$-component]. What are your $x$ and $y$ velocities $\dot{x}$ and $\dot{y}$? You solution will be in terms of $a$, the altitude, and $b$, the flow coefficient.
