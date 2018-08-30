# Project {#ukf-project status=notready}

## Background
Before we dive into the UKF, there are some foundations that we should build up:

1. The Bayes Filter

2. Gaussians

3. The Kalman Filter

### The Bayes Filter

The Bayes Filter is the mathematical basis for the Kalman Filter and other probabilistic robotics algorithms such as Monte Carlo Localization, which you will learn about and implement in a later project. The Bayes Filter algorithm incorporates information available to the robot at each point in time to produce an accurate estimate of the robot’s position.

The core idea of the Bayes filter is to take advantage of data from two sources of information: the controls given to the robot and the measurements detected by the robot's sensors.

At each new time step, the Bayes filter recursively produces a state estimate, represented as a probability density function called the *belief.* The belief assigns to every possible pose in the state space of the robot the probability that it is the robot's true location. This probability is found in two steps called **prediction** and **update**.

The prediction step incorporates controls given to the robot between the previous state and the current one. It finds the probability of reaching a new state given the previous state and the control (hence recursion). The model used to find this probability is known as a *state transition model* and is specific to the robot in question.

The state transition model:
$p(x_{t}|u_{t},x_{t-1})$
 the probability that the most recent control $u_t$ will transition the previous state $x_{t-1}$ to the current state $x_t$

It is possible to estimate the state of the robot using only the prediction step and not incorporating the measurements taken by the robot's sensors. This is known as *dead reckoning*. The dead reckoning estimate may be made more accurate by incorporating measurements from the robot's sensors.

The Bayes filter does this in the update step by finding the probability that the current measurements are observed in the current state. The model used for this is known as a *measurement model* and is specific to the robot in question.

The measurement model:
$p(z_{t}|x_{t})$
the probability that the current measurement $z_t$ is observed given the state $x_t$

You may have noticed that each of the above steps required computing a probability stated like "the probability of x given y." Such a probability is denoted $p(X|Y)$ and may be calculated by the famous Bayes Theorem for conditional probabilities, hence the name of the algorithm.

Now, let's take a look at the Bayes Filter:

$\hspace{5mm} \text{Bayes_Filter}(bel(x_{t-1}), u_{t}, z_{t}):$

$\hspace{10mm} \text{for all } x_t \text{ do}:$

$\hspace{15mm} \bar{bel}(x_t) = \int p(x_{t}|u_{t},x_{t-1})bel(x_{t-1})dx$

$\hspace{15mm} bel(x_t) = \eta p(z_{t}|x_{t})\bar{bel}(x_t)$

$\hspace{10mm} \text{endfor}$

$\hspace{10mm} \text{return } bel(x_{t})$
	 
ie compute a belief by finding the probability of each possible new state. For each state, incorporate both the probability that the control transitions the previous state to this one and that the current measurements are observed in this state.

The first step $\bar{bel}(x_t) = \int p(x_{t}|u_{t},x_{t-1})bel(x_{t-1})dx$  is the *motion prediction*. $\bar{bel}(x_t)$ represents the belief *before* the measurement is incorporated. The integral is computed discretely and becomes:
$\sum_x{p(x_t|u_t,x_{t-1})bel(x_{t-1})}$

The second step $bel(x_t) = \eta p(z_{t}|x_{t})\bar{bel}(x_t)$ is the *measurement update*. This computation is straightforward, the normalizer $\eta$ is the reciprocal of the sum of $p(z_{t}|x_{t})\bar{bel}(x_t)$ over all $x_t$. This factor will normalize the sum.

The following diagram illustrates how a robot uses a Bayes Filtering algorithm to compute a belief distribution over time [](#bib:probabilistic_robotics):

![robot_hallway](probabilistic_robotics_robot_hallway.png)

Each of the below steps corresponds to a labeled part of the diagram:

(a) Start, the robot has a uniform belief distribution

(b) The robot measures a door, and believes it is equally likely to be in each of three positions, note the non-zero probability everywhere else

(c) The robot moves, shifting the belief distribution and reducing certainty

(d) The robot measures a second door, and now is very certain of its position

(e) The belief distribution shifts again, and the certainty decreases

The takeaway from the above? Motion *decreases* certainty and measurement *increases* certainty.

### Gaussians

**TODO: Brief description of Gaussians as needed to discuss the Kalman Filter. Perhaps show multivariate Gaussians as well, e.g., with a 3D plot and an error ellipse to illustrate 2D Gaussian**

### The Kalman Filter
#### High-Level Description of the Kalman Filter Algorithm

The Kalman Filter's main goal is to fuse measurement readings with predicted states. For example, if we know (with some degree of uncertainty) that the drone is moving upward, then this knowledge can inform us about the drone's position at the next time step. We can form a **prediction** of the drone's state at the next time step given the drone's current state and any control inputs that we give the drone. Then, at the next time step when we receive a new measurement, we can perform an **update** of our state estimate. In this update step, we look at the difference between the new measurement and the predicted state. This difference is known as the *residual*. Our new state estimate (referred to in literature as the *posterior*) lies somewhere between the predicted state, or *prior*, and the measurement; the scaling factor that accomplishes this is known as the *Kalman gain*. Figure 1 depicts this process of prediction and measurement update.

![residual_chart](residual_chart.png)

*Figure 1. Interaction of the Prediction and Measurement in a Kalman Filter [](#bib:labbe_kalman)*

##### State Vector and Covariance Matrix

The KF accomplishes its state estimate by tracking certain **state variables** in a state vector, such as position and velocity along an axis, and the **covariance matrix** corresponding to the state vector. For example, a state vector that tracks the drone's position and velocity along the z-axis would look like:

$$\mathbf{x}=\begin{bmatrix}
z \\
\dot z
\end{bmatrix}$$

where $z$ and $\dot z$ are the position and velocity of the drone along the z-axis, respectively. The state vector tracks the mean $\mu$ of each state variable, which we assume is normally distributed about $\mu$. To characterize the uncertainty in this state estimate, we use an $n \times n$ covariance matrix where $n$ is the size of the state vector. For this state vector, then, we define the covariance matrix as:

$$\mathbf{P}=\begin{bmatrix}
\sigma^2_z & \sigma_{z,\dot z} \\
\sigma_{\dot z,z} & \sigma^2_{\dot z}
\end{bmatrix}$$

where $\sigma^2_z = \text{Var}\left( z \right)$, for example, denotes the variance in the position estimates and $\sigma_{\dot z,z} = \sigma_{z,\dot z} = \text{Cov}\left( z, \dot z \right)$ denotes the covariance between the position and velocity estimates. Typically, position and velocity are positively correlated, as a positive velocity indicates that the drone will likely be at a more positive position at the next time step.

**TODO: Write out the recursive algorithm. Probably refer to and cite** ***Probabilistic Robotics***

#### A Closer Look at Some of the Math of the Kalman Filter

**TODO: Figure out what to put here**

---

## Designing and Implementing a Kalman Filter

To apply a Kalman Filter to a specific robot, there are certain parts of the algorithm that we need to define. The first aspect of the KF design process specific to the robot application is the selection of state variables to track in the state vector. Additionally, the motion model of the robot and its sensor suite demand careful thought when designing a KF: the former determines the state transition function, and the latter specifies the measurement function. Furthermore, the process noise and measurement covariance matrices must be determined. The filter must then be initialized and sometimes adapted to handle asynchronous inputs from real-world sensors. Finally, once a filter is implemented, it is a good idea to tune and test it in simulation and then on the real robot, quantifying its performance if possible. We will be going over these design decisions and implementation details in this section.

#### State Vector
**TODO**

#### State Transition Function
**TODO**

#### Measurement Function
**TODO**

#### Process Noise and Measurement Covariance Matrices
**TODO**

#### Initialize the Filter
**TODO**

#### Asynchronous Inputs
**TODO**

#### Tune and Test the Filter
**TODO: Write this section and include description on quantitatively evaluating the performance of the filter**

---

## The Unscented Kalman Filter: Nonlinear State Estimation
**TODO**

---

## Our Implementation: UKF in One Spatial Dimension
**TODO**

---

## Our Implementation: UKF in Three Spatial Dimensions
**TODO**


